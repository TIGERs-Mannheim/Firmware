#pragma once

#include "net/udp_socket.h"
#include "math/ema_filter.h"
#include "util/shell_cmd.h"
#include "protobuf-c/protobuf-c.h"

#define SSL_VISION_MAX_CAMS 16
#define SSL_VISION_MAX_ROBOTS 32

// protoc allocator size for unpacking
#define SSL_VISION_ALLOC_BUF_SIZE 8192

typedef struct _SslVisionObject
{
	uint8_t isRecentlyDetected;

	systime_t tLastDetection; // Based on OS clock, only precise to 250us
	uint32_t camIdOfLastDetection;

	uint32_t tCapture_us; // Based on SysTimeUSec, compensated for vision processing and network delay
	float position_m[2];
	float orientation_rad;

	mutex_t accessMtx;

	uint32_t detectionsUsed;
	uint32_t detectionsOutlier;
	uint32_t detectionsOutOfViewport;
} SslVisionObject;

typedef struct _SslVisionCameraViewport
{
	float xRange_m[2];
	float yRange_m[2];
} SslVisionCameraViewport;

typedef struct _SslVisionCamera
{
	uint8_t hasCalibration;

	uint32_t tLastReception_us;
	EMAFilter detectionRate_Hz;
	IPv4Address sourceIp;
	uint32_t lastDetectionFrameNumber;
	uint32_t missedDetectionFrames;

	SslVisionCameraViewport viewport;
} SslVisionCamera;

typedef struct _SslVisonGeometry
{
	float fieldLength_m; // Extent along the X axis
	float fieldWidth_m; // Extent along the Y axis
	float goalWidth_m;
	float goalDepth_m;
	float boundaryWidth_m; // Extra space beyond field lines
} SslVisionGeometry;

typedef struct _SslVisionData
{
	NetIf* pIf;
	IPv4Address ip;
	uint16_t port;

	uint32_t changeDetectionCameraDelay_ms;
	float robotOutlierMaxVelXY_mDs;
	float robotOutlierMaxVelW_radDs;
	float ballOutlierMaxVelXY_mDs;
	uint32_t objectTimeout_ms;
	uint32_t visionTimeout_ms;
} SslVisionData;

typedef struct _SslVision
{
	SslVisionData data;

	uint8_t pAllocBuf[SSL_VISION_ALLOC_BUF_SIZE];
	size_t allocUsed;

	ProtobufCAllocator allocator;

	UDPSocket socket;

	uint8_t isOnline;
	systime_t tLastReception;

	SslVisionObject robots[SSL_VISION_MAX_ROBOTS];
	SslVisionObject ball;
	SslVisionCamera cams[SSL_VISION_MAX_CAMS];
	SslVisionGeometry geometry;

	uint32_t parsedPackets;
	uint32_t parseErrors;
	uint32_t outOfMemoryErrors;

	ShellCmdHandler cmdHandler;

	THD_WORKING_AREA(waTask, 4096);
	thread_t* pTask;
} SslVision;

void SslVisionInit(SslVision* pVision, SslVisionData* pInit, tprio_t prio);
void SslVisionUpdateViewport(SslVision* pVision, uint32_t camId, const SslVisionCameraViewport* pNewViewport);
