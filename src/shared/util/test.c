#include "test.h"
#include <string.h>
#include <stdio.h>

#define TEST_OBJ_POOL_SIZE 10

static void testTask(void* params);
static void testProgressTask(void* params);

typedef struct _RegisteredTest
{
	uint32_t id;
	TestFunction func;
	size_t argSize;
} RegisteredTest;

static struct _TestData
{
	Test objPool[TEST_OBJ_POOL_SIZE];
	msg_t msgBuffer[TEST_OBJ_POOL_SIZE];
	objects_fifo_t objectFifo;

	RegisteredTest functions[TEST_MAX_FUNCTIONS];

	size_t functionsUsed;

	THD_WORKING_AREA(waTask, 4096);
	thread_t* pTask;

	THD_WORKING_AREA(waProgressTask, 1024);
	thread_t* pProgressTask;
} test;

void TestInit(tprio_t taskPrio)
{
	chFifoObjectInit(&test.objectFifo, sizeof(Test), TEST_OBJ_POOL_SIZE, test.objPool, test.msgBuffer);

	chThdCreateStatic(test.waTask, sizeof(test.waTask), taskPrio, &testTask, 0);
}

void TestRegister(uint32_t id, TestFunction func, size_t argSize, size_t resultSize)
{
	if(test.functionsUsed >= TEST_MAX_FUNCTIONS)
	{
		chSysHalt("Maximum number of test functions exceeded");
	}

	chDbgAssert(argSize <= TEST_MAX_ARGUMENT_SIZE, "Test argument size too large");
	chDbgAssert(resultSize <= TEST_MAX_RESULT_SIZE, "Test result size too large");

	test.functions[test.functionsUsed].id = id;
	test.functions[test.functionsUsed].func = func;
	test.functions[test.functionsUsed].argSize = argSize;
	test.functionsUsed++;
}

Test* TestScheduleWithUpdates(uint32_t id, const void* pArg, uint8_t autoFree, UpdateFunc updateFunc, void* pUser)
{
	Test* pTest = (Test*)chFifoTakeObjectTimeout(&test.objectFifo, TIME_IMMEDIATE);
	if(!pTest)
		return pTest;

	uint8_t idFound = 0;
	for(size_t i = 0; i < test.functionsUsed; i++)
	{
		if(test.functions[i].id == id)
		{
			memcpy(pTest->argumentData, pArg, test.functions[i].argSize);
			idFound = 1;
		}
	}

	if(!idFound)
	{
		fprintf(stderr, "Unknwon test with ID: %u\r\n", id);
		return 0;
	}

	pTest->id = id;
	pTest->autoFree = autoFree;
	pTest->state = TEST_STATE_QUEUED;
	pTest->expectedTestTime_s = 1.0f;
	pTest->pUpdateFunc = updateFunc;
	pTest->pUser = pUser;

	chFifoSendObject(&test.objectFifo, pTest);

	if(pTest->autoFree)
		return 0;

	return pTest;
}

Test* TestSchedule(uint32_t id, const void* pArg, uint8_t autoFree)
{
	return TestScheduleWithUpdates(id, pArg, autoFree, 0, 0);
}

float TestGetProgress(const Test* pTest)
{
	if(pTest->state == TEST_STATE_DONE)
		return 100.0f;

	if(pTest->state != TEST_STATE_ACTIVE)
		return 0.0f;

	uint32_t elapsedTime_ms = TIME_I2MS(chVTTimeElapsedSinceX(pTest->tStart));
	float percentComplete = (elapsedTime_ms * 0.1) / pTest->expectedTestTime_s;

	if(percentComplete > 99.0f)
		percentComplete = 99.0f;

	return percentComplete;
}

void* TestGetArgument(Test* pTest)
{
	if(pTest->state == TEST_STATE_UNUSED)
		return 0;

	return pTest->argumentData;
}

void* TestGetResult(Test* pTest)
{
	if(pTest->state == TEST_STATE_UNUSED)
		return 0;

	return pTest->resultData;
}

void TestFree(Test* pTest)
{
	chDbgAssert(pTest->state == TEST_STATE_DONE, "Invalid test state");

	pTest->state = TEST_STATE_UNUSED;
	chFifoReturnObject(&test.objectFifo, pTest);
}

static void testTask(void* params)
{
	(void)params;

	chRegSetThreadName("TestExec");

	while(1)
	{
		Test* pTest = 0;
		chFifoReceiveObjectTimeout(&test.objectFifo, (void**)&pTest, TIME_INFINITE);

		uint8_t testExecuted = 0;

		for(size_t i = 0; i < test.functionsUsed; i++)
		{
			if(test.functions[i].id == pTest->id)
			{
				pTest->tStart = chVTGetSystemTime();
				pTest->state = TEST_STATE_ACTIVE;

				test.pProgressTask = chThdCreateStatic(test.waProgressTask, sizeof(test.waProgressTask), LOWPRIO, &testProgressTask, pTest);

				(*test.functions[i].func)(pTest);

				pTest->state = TEST_STATE_DONE;
				testExecuted = 1;

				chThdWait(test.pProgressTask);

				if(pTest->pUpdateFunc)
					(*pTest->pUpdateFunc)(pTest, pTest->pUser);

				if(pTest->autoFree)
					TestFree(pTest);

				break;
			}
		}

		if(!testExecuted)
		{
			fprintf(stderr, "Unknown test ID: %u\r\n", pTest->id);
		}
	}
}


static void testProgressTask(void* params)
{
	Test* pTest = (Test*)params;

	chRegSetThreadName("TestProgress");

	while(pTest->state == TEST_STATE_ACTIVE)
	{
		if(pTest->pUpdateFunc)
		{
			(*pTest->pUpdateFunc)(pTest, pTest->pUser);
		}

		chThdSleepMilliseconds(50);
	}

	chThdExit(0);
}
