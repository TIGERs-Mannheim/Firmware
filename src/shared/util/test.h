#pragma once

#include "ch.h"

#define TEST_MAX_FUNCTIONS	32
#define TEST_MAX_ARGUMENT_SIZE 16
#define TEST_MAX_RESULT_SIZE 32

#define TEST_STATE_UNUSED	0
#define TEST_STATE_QUEUED	1
#define TEST_STATE_ACTIVE	2
#define TEST_STATE_DONE		3


typedef struct _Test
{
	uint32_t id;
	uint8_t argumentData[TEST_MAX_ARGUMENT_SIZE];
	uint8_t resultData[TEST_MAX_RESULT_SIZE];

	void(*pUpdateFunc)(struct _Test*, void*);
	void* pUser;

	systime_t tStart;
	float expectedTestTime_s;
	int16_t state;

	uint8_t autoFree; // get progress and get result only possible with autoFree = 0
} Test;

typedef void(*TestFunction)(Test*);
typedef void(*UpdateFunc)(Test*, void*);

/**
 * Initialize test system.
 *
 * @param taskPrio Priority for test execution.
 */
void TestInit(tprio_t taskPrio);

/**
 * Register a new test function.
 *
 * @param id
 * @param func
 * @param argSize Sizeof function argument object
 * @param resultSize Sizeof function result object
 */
void TestRegister(uint32_t id, TestFunction func, size_t argSize, size_t resultSize);

/**
 * Schedule a test to be executed.
 *
 * @param id Test ID
 * @param pArg Argument object to test function containing parameters
 * @param autoFree Free test object when test is complete
 * @return Pointer to created test object or zero if no slot is free or autoFree == 1
 */
Test* TestSchedule(uint32_t id, const void* pArg, uint8_t autoFree);

/**
 * Schedule a test to be executed with a periodic update function being called.
 *
 * @param id Test ID
 * @param pArg Argument object to test function containing parameters
 * @param autoFree Free test object when test is complete
 * @param updateFunc Regularly called update function
 * @param pUser User point to update function
 * @return Pointer to created test object or zero if no slot is free or autoFree == 1
 */
Test* TestScheduleWithUpdates(uint32_t id, const void* pArg, uint8_t autoFree, UpdateFunc updateFunc, void* pUser);

/**
 * Get percentage of test being complete.
 * @param pTest
 * @return 0.0 - 100.0%
 */
float TestGetProgress(const Test* pTest);

/**
 * Get argument data of test. It is expected the calling function knows what this is.
 * @param pTest
 */
void* TestGetArgument(Test* pTest);

/**
 * Get result data of test. It is expected the calling function knows what this is.
 * @param pTest
 */
void* TestGetResult(Test* pTest);

/**
 * Return a test object back to the pool.
 *
 * @param pTest
 */
void TestFree(Test* pTest);
