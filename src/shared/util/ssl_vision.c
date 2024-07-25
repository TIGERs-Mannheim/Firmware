#include "ssl_vision.h"
#include "commands.h"
#include "hal/sys_time.h"
#include "math/angle_math.h"
#include "protobuf-c/messages_robocup_ssl_wrapper.pb-c.h"
#include <math.h>

#define EVENT_MASK_SOCKET_RX	EVENT_MASK(0)

static void sslVisionTask(void* pParam);
static void handleWrapperPacket(SslVision* pVision, NetPkt* pPkt);
static uint8_t isInViewport(SslVision* pVision, uint32_t camId, float x_m, float y_m);
static void updateRobotDetections(SslVision* pVision, uint32_t camId, uint32_t tCapture_us, SSLDetectionRobot** ppRobots, size_t numRobots, uint32_t idOffset);
static void updateObjectDetection(SslVision* pVision, SslVisionObject* pObj, float x_mm, float y_mm, float orient_rad, uint32_t tCapture_us, uint8_t camId, float maxVelXY_mDs, float maxVelW_radDs);
static void* allocMem(void* pUser, size_t size);
static void freeMem(void*, void*);
static void registerShellCommands(ShellCmdHandler* pHandler);

void SslVisionInit(SslVision* pVision, SslVisionData* pInit, tprio_t prio)
{
	pVision->data = *pInit;

	for(size_t i = 0; i < SSL_VISION_MAX_CAMS; i++)
	{
		EMAFilterInit(&pVision->cams[i].detectionRate_Hz, 0.95f);

		pVision->cams[i].viewport.xRange_m[0] = -INFINITY;
		pVision->cams[i].viewport.xRange_m[1] = INFINITY;
		pVision->cams[i].viewport.yRange_m[0] = -INFINITY;
		pVision->cams[i].viewport.yRange_m[1] = INFINITY;
	}

	for(size_t i = 0; i < SSL_VISION_MAX_ROBOTS; i++)
	{
		chMtxObjectInit(&pVision->robots[i].accessMtx);
	}

	chMtxObjectInit(&pVision->ball.accessMtx);

	ShellCmdHandlerInit(&pVision->cmdHandler, pVision);
	registerShellCommands(&pVision->cmdHandler);

	pVision->allocator.alloc = &allocMem;
	pVision->allocator.free = &freeMem;
	pVision->allocator.allocator_data = pVision;

	UDPSocketOpen(&pVision->socket, pInit->pIf, pInit->port, pInit->ip);

	pVision->pTask = chThdCreateStatic(pVision->waTask, sizeof(pVision->waTask), prio, &sslVisionTask, pVision);
}

void SslVisionUpdateViewport(SslVision* pVision, uint32_t camId, const SslVisionCameraViewport* pNewViewport)
{
	if(camId > SSL_VISION_MAX_CAMS)
		return;

	memcpy(&pVision->cams[camId].viewport, pNewViewport, sizeof(SslVisionCameraViewport));
}

static void sslVisionTask(void* pParam)
{
	SslVision* pVision = pParam;

	chRegSetThreadName("SSL_VISION");

	event_listener_t udpListener;

	chEvtRegisterMask(&pVision->socket.eventSource, &udpListener, EVENT_MASK_SOCKET_RX);

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));

		if(events & EVENT_MASK_SOCKET_RX)
		{
			while(1)
			{
				NetPkt* pPkt = UDPSocketReceive(&pVision->socket);
				if(!pPkt)
					break;

				handleWrapperPacket(pVision, pPkt);

				UDPSocketFree(&pVision->socket, pPkt);
			}
		}

		if(pVision->isOnline && chVTTimeElapsedSinceX(pVision->tLastReception) > TIME_MS2I(pVision->data.visionTimeout_ms))
			pVision->isOnline = 0;

		for(size_t i = 0; i < SSL_VISION_MAX_ROBOTS; i++)
		{
			SslVisionObject* pRobot = &pVision->robots[i];

			if(pRobot-> isRecentlyDetected && chVTTimeElapsedSinceX(pRobot->tLastDetection) > TIME_MS2I(pVision->data.objectTimeout_ms))
				pRobot->isRecentlyDetected = 0;
		}

		if(pVision->ball.isRecentlyDetected && chVTTimeElapsedSinceX(pVision->ball.tLastDetection) > TIME_MS2I(pVision->data.objectTimeout_ms))
			pVision->ball.isRecentlyDetected = 0;
	}
}

static void handleWrapperPacket(SslVision* pVision, NetPkt* pPkt)
{
	uint8_t* pData = NetBufGetHead(&pPkt->buf);
	size_t length = NetBufGetSizeUsed(&pPkt->buf);

	pVision->allocUsed = 0;	// free vision buffer

	// Protobuf unpacking
	SSLWrapperPacket* pPacket = 0;
	pPacket = ssl__wrapper_packet__unpack(&pVision->allocator, length, pData);
	if(pPacket == 0)
	{
		pVision->parseErrors++;
		return;
	}

	pVision->isOnline = 1;
	pVision->tLastReception = chVTGetSystemTimeX();
	pVision->parsedPackets++;

	// Process geometry, if present
	if(pPacket->geometry != 0)
	{
		SSLGeometryFieldSize* field = pPacket->geometry->field;
		pVision->geometry.boundaryWidth_m = field->boundary_width * 0.001f;
		pVision->geometry.fieldLength_m = field->field_length * 0.001f;
		pVision->geometry.fieldWidth_m = field->field_width * 0.001f;
		pVision->geometry.goalDepth_m = field->goal_depth * 0.001f;
		pVision->geometry.goalWidth_m = field->goal_width * 0.001f;

		for(size_t i = 0; i < pPacket->geometry->n_calib; i++)
		{
			uint32_t camId = pPacket->geometry->calib[i]->camera_id;
			if(camId < SSL_VISION_MAX_CAMS)
				pVision->cams[camId].hasCalibration = 1;
		}
	}

	if(pPacket->detection == 0)
		return;

	// Process common camera data
	float procDelay_s = (float)(pPacket->detection->t_sent - pPacket->detection->t_capture);
	procDelay_s += 0.001f;	// 1ms transport delay
	if(procDelay_s < 0)
		procDelay_s = 0;

	uint32_t tNow_us = SysTimeUSec();
	uint32_t procDelay_us = (uint32_t)(procDelay_s*1e6f);
	uint32_t tCapture_us = tNow_us - procDelay_us;
	uint32_t camId = pPacket->detection->camera_id;

	if(camId >= SSL_VISION_MAX_CAMS)
		return;

	SslVisionCamera* pCam = &pVision->cams[camId];
	pCam->sourceIp = pPkt->ipv4.src;

	uint32_t dt_us = tNow_us - pCam->tLastReception_us;
	pCam->tLastReception_us = tNow_us;
	EMAFilterUpdate(&pCam->detectionRate_Hz, 1e6f/(float)dt_us);

	if(pCam->lastDetectionFrameNumber == 0)
		pCam->lastDetectionFrameNumber = pPacket->detection->frame_number;

	uint32_t frameDiff = pPacket->detection->frame_number - pCam->lastDetectionFrameNumber;
	if(frameDiff > 1)
		pCam->missedDetectionFrames += frameDiff-1;

	pCam->lastDetectionFrameNumber = pPacket->detection->frame_number;

	// Process robot detections
	updateRobotDetections(pVision, camId, tCapture_us, pPacket->detection->robots_yellow, pPacket->detection->n_robots_yellow, 0);
	updateRobotDetections(pVision, camId, tCapture_us, pPacket->detection->robots_blue, pPacket->detection->n_robots_blue, CMD_BOT_COUNT_HALF);

	// Process ball detections
	SSLDetectionBall* pClosestBall = 0;
	float closestDistanceSquared_m = INFINITY;

	for(size_t i = 0; i < pPacket->detection->n_balls; i++)
	{
		SSLDetectionBall* pBall = pPacket->detection->balls[i];
		float pos_m[2] = { pBall->x * 0.001f, pBall->y * 0.001f };
		float diff_m[2] = { pos_m[0] - pVision->ball.position_m[0], pos_m[1] - pVision->ball.position_m[1] };
		float distSquared = diff_m[0] * diff_m[0] + diff_m[1] * diff_m[1];

		if(distSquared < closestDistanceSquared_m)
		{
			closestDistanceSquared_m = distSquared;
			pClosestBall = pBall;
		}
	}

	if(pClosestBall)
	{
		updateObjectDetection(pVision, &pVision->ball, pClosestBall->x, pClosestBall->y, 0.0f, tCapture_us, camId, pVision->data.ballOutlierMaxVelXY_mDs, 1.0f);
	}
}

static void updateRobotDetections(SslVision* pVision, uint32_t camId, uint32_t tCapture_us, SSLDetectionRobot** ppRobots, size_t numRobots, uint32_t idOffset)
{
	for(size_t i = 0; i < numRobots; i++)
	{
		SSLDetectionRobot* pRobot = ppRobots[i];

		if(pRobot->has_robot_id == 0 || pRobot->has_orientation == 0)
			continue;

		if(pRobot->robot_id >= SSL_VISION_MAX_ROBOTS)
			continue;

		SslVisionObject* pObj = &pVision->robots[pRobot->robot_id + idOffset];

		updateObjectDetection(pVision, pObj, pRobot->x, pRobot->y, pRobot->orientation, tCapture_us, camId,
				pVision->data.robotOutlierMaxVelXY_mDs, pVision->data.robotOutlierMaxVelW_radDs);
	}
}

static void updateObjectDetection(SslVision* pVision, SslVisionObject* pObj, float x_mm, float y_mm, float orient_rad, uint32_t tCapture_us, uint8_t camId,
		float maxVelXY_mDs, float maxVelW_radDs)
{
	const float posX_m = x_mm * 0.001f;
	const float posY_m = y_mm * 0.001f;

	// Discard this detection if it is out of the cameras viewport
	if(!isInViewport(pVision, camId, posX_m, posY_m))
	{
		pObj->detectionsOutOfViewport++;
		return;
	}

	// Discard this detection if it is from a different camera than last time (e.g. camera overlap area)
	if(pObj->camIdOfLastDetection != camId && chVTTimeElapsedSinceX(pObj->tLastDetection) < TIME_MS2I(pVision->data.changeDetectionCameraDelay_ms))
		return;

	// Outlier filter
	const float visionDt_s = (tCapture_us - pObj->tCapture_us)*1e-6f;

	const float searchRadiusXY_m = maxVelXY_mDs  * visionDt_s;
	const float searchRadiusW_rad = maxVelW_radDs * visionDt_s;

	float diffX_m = posX_m - pObj->position_m[0];
	float diffY_m = posY_m - pObj->position_m[1];
	float diffXY_m = sqrtf(diffX_m*diffX_m + diffY_m*diffY_m);
	float diffW_rad = AngleNormalize(orient_rad - pObj->orientation_rad);

	if(diffXY_m > searchRadiusXY_m || fabsf(diffW_rad) > searchRadiusW_rad)
	{
		pObj->detectionsOutlier++;
		return;
	}

	pObj->detectionsUsed++;

	chMtxLock(&pObj->accessMtx);
	pObj->isRecentlyDetected = 1;
	pObj->tLastDetection = chVTGetSystemTimeX();
	pObj->camIdOfLastDetection = camId;
	pObj->position_m[0] = posX_m;
	pObj->position_m[1] = posY_m;
	pObj->orientation_rad = orient_rad;
	pObj->tCapture_us = tCapture_us;
	chMtxUnlock(&pObj->accessMtx);
}

static uint8_t isInViewport(SslVision* pVision, uint32_t camId, float x_m, float y_m)
{
	if(camId >= SSL_VISION_MAX_CAMS)
		return 1;

	const SslVisionCameraViewport* pViewport = &pVision->cams[camId].viewport;

	if(x_m >= pViewport->xRange_m[0] && x_m <= pViewport->xRange_m[1] && y_m >= pViewport->yRange_m[0] && y_m <= pViewport->yRange_m[1])
		return 1;

	return 0;
}

static void* allocMem(void* pUser, size_t size)
{
	SslVision* pVision = pUser;

	if(pVision->allocUsed + size > SSL_VISION_ALLOC_BUF_SIZE)
	{
		pVision->outOfMemoryErrors++;
		return 0;
	}

	void* pMem = pVision->pAllocBuf + pVision->allocUsed;

	size_t pad = (4-(size%4))%4;

	pVision->allocUsed += size+pad;

	return pMem;
}

static void freeMem(void*, void*)
{
}

SHELL_CMD(objects, "List detected robots and a ball");

SHELL_CMD_IMPL(objects)
{
	(void)argc; (void)argv;
	SslVision* pVision = pUser;

	printf("  ID       X       Y      W  Cam  detUsed  detOutV  detFilt\r\n");

	for(size_t i = 0; i < SSL_VISION_MAX_ROBOTS; i++)
	{
		SslVisionObject* pObj = &pVision->robots[i];
		if(!pObj->isRecentlyDetected)
			continue;

		size_t id = i;
		char team = 'Y';
		if(id >= CMD_BOT_COUNT_HALF)
		{
			id -= CMD_BOT_COUNT_HALF;
			team = 'B';
		}

		printf(" %2u%c % 7.3f % 7.3f % 6.3f   %2u %8u %8u %8u\r\n", id, team, pObj->position_m[0], pObj->position_m[1], pObj->orientation_rad,
				pObj->camIdOfLastDetection, pObj->detectionsUsed, pObj->detectionsOutOfViewport, pObj->detectionsOutlier);
	}

	SslVisionObject* pObj = &pVision->ball;
	if(pObj->isRecentlyDetected)
	{
		printf("Ball % 7.3f % 7.3f          %2u %8u %8u %8u\r\n", pObj->position_m[0], pObj->position_m[1],
				pObj->camIdOfLastDetection, pObj->detectionsUsed, pObj->detectionsOutOfViewport, pObj->detectionsOutlier);
	}
}

SHELL_CMD(cameras, "List active cameras");

SHELL_CMD_IMPL(cameras)
{
	(void)argc; (void)argv;
	SslVision* pVision = pUser;

	printf("ID   Rate             IP    FMiss   XMin   XMax   YMin   YMax\r\n");

	for(size_t i = 0; i < SSL_VISION_MAX_CAMS; i++)
	{
		SslVisionCamera* pCam = &pVision->cams[i];
		if(!pCam->hasCalibration)
			continue;

		printf("%2u %6.2f  %s %8u % 6.3f % 6.3f % 6.3f % 6.3f\r\n", i, pCam->detectionRate_Hz.value, IPv4FormatAddress(pCam->sourceIp), pCam->missedDetectionFrames,
				pCam->viewport.xRange_m[0], pCam->viewport.xRange_m[1], pCam->viewport.yRange_m[0], pCam->viewport.yRange_m[1]);
	}
}

SHELL_CMD(geom, "Show received geometry");

SHELL_CMD_IMPL(geom)
{
	(void)argc; (void)argv;
	SslVision* pVision = pUser;

	printf("Field (LxW): %.3fm x %.3fm\r\n", pVision->geometry.fieldLength_m, pVision->geometry.fieldWidth_m);
	printf("Goal (WxD):  %.3fm x %.3fm\r\n", pVision->geometry.goalWidth_m, pVision->geometry.goalDepth_m);
	printf("Boundary width: %.3fm\r\n", pVision->geometry.boundaryWidth_m);
}

SHELL_CMD(stats, "Show packet statistics");

SHELL_CMD_IMPL(stats)
{
	(void)argc; (void)argv;
	SslVision* pVision = pUser;

	printf("Vision: %s\r\n", (pVision->isOnline ? "online" : "offline"));
	printf("Packets: %u parsed, %u errornous, %u out of memory\r\n", pVision->parsedPackets, pVision->parseErrors, pVision->outOfMemoryErrors);
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, objects_command);
	ShellCmdAdd(pHandler, cameras_command);
	ShellCmdAdd(pHandler, geom_command);
	ShellCmdAdd(pHandler, stats_command);
}
