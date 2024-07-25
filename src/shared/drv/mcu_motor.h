#pragma once

#include "log_msgs.h"
#include "ch.h"
#include "hal/uart_fifo.h"
#include "util/st_bootloader.h"
#include "util/device_profiler.h"
#include "util/config.h"
#include "util/shell_cmd.h"

#define MCU_MOTOR_EVENT_DATA_RECEIVED	EVENT_MASK(0)
#define MCU_MOTOR_EVENT_BREAK_DETECTED	EVENT_MASK(1)

typedef struct PACKED _MotorParams
{
	float Km;	// motor torque constant [Nm/A]
	float Kn;	// motor speed constant [rad/(s*V)]
	float Ke;	// motor back-EMF constant [V/rad]
	float Kf;	// motor friction constant [Nm*s]
	float R;	// motor resistance [Ohm]
	float L;	// motor inductance [H]
	float pp;	// motor pole pairs
} MotorParams;

typedef struct PACKED _McuMotorFocConfig
{
	float curDKp;
	float curDKi;
	float curQKp;
	float curQKi;
} McuMotorFocConfig;

typedef struct PACKED _McuMotorVelConfig
{
	float velKp;
	float velKi;
	float velAntiJitter;
} McuMotorVelConfig;

typedef struct _McuMotorCtrlParams
{
	float speedRawToRadPerSec; // speedRaw can be encoder delta or model speed (if no encoder installed)
	float curGainRealToMotor;  // depends on motor controller firmware
	float velGainRealToMotor;  // depends on motor controller firmware
	float maxCurrent_A;
} McuMotorCtrlParams;

typedef struct _McuMotorMeasurement
{
	uint32_t timestamp_us;

	float cpuLoad;
	float avgSupplyVoltage1ms_V;
	float avgTemperature1ms_degC;
	float avgCurrentUVW1ms_A[3];
	float avgCurrentDQ1ms_A[2];
	float avgVoltageDQ1ms_V[2];
	float currentOffset_A;
	float encoderVelocity_radDs; // motor [rad/s]
	float hallVelocity_radDs; // motor [rad/s]

	uint8_t hallPos;
	int32_t hallComTime;
	uint16_t hallInvalidTransitions;

	uint16_t encPos; // electrical position in encoder ticks
	int16_t encDelta; // ticks within 1ms
	int16_t encCorrection; // -32768 to 32767 (one electrical revolution)
	uint16_t encTicks; // ticks per revolution, should be fix if nothing is broken
	uint16_t encPrescaler; // encoder prescaler in interpolation mode (no encoder installed)

	uint16_t flags;
} McuMotorMeasurement;

typedef struct _McuMotor
{
	UARTFifo* pUart;

	STBootloader bootloader;
	STBootloaderFlashResult flashResult;
	int16_t flashFuncResult;
	const uint8_t* pFwProgram;
	uint32_t fwSize;

	uint8_t doFwUpdate;

	DeviceProfiler profiler;

	ShellCmdHandler cmdHandler;

	THD_WORKING_AREA(waTask, 512);
	thread_t* pTask;
	char taskName[10];

	event_source_t eventSource;

	// configuration
	uint8_t motorId;
	const McuMotorFocConfig* pFocConfig;
	const McuMotorVelConfig* pVelConfig;
	const MotorParams* pMotorParams;
	const McuMotorCtrlParams* pCtrlParams;

	// input from motor
	McuMotorMeasurement meas;
	mutex_t measMutex;

	// debug data recording
	int16_t* pRecordData;
	size_t recordsMax;
	uint8_t recordMode;
	uint8_t recordOneShot;
	uint32_t recordCounter;
	uint8_t breakTriggered;
	uint32_t totalBreaks;

	uint8_t debugInhibitTx;

	// output to motor
	uint8_t mode;
	float setVoltage_V[2]; // [V]
	float setCurrent_A[2]; // [A]
	float setVelocity_radDs; // [rad/s]
	uint32_t txTimestamp_us;
	uint8_t hallOnly;
	mutex_t outMutex;
} McuMotor;

typedef struct _McuMotorData
{
	uint8_t motorId;
	UARTFifo* pUart;
	GPIOPinInterface* pRstPin;
	GPIOPinInterface* pBootPin;
	const uint8_t* pFwProgram;
	uint32_t fwSize;
	const MotorParams* pMotorParams;
	const McuMotorCtrlParams* pCtrlParams;
	const McuMotorFocConfig* pFocConfig;
	const McuMotorVelConfig* pVelConfig;
	int16_t* pDebugRecordData;
	size_t debugRecordSize;
} McuMotorData;

void McuMotorInit(McuMotor* pMot, McuMotorData* pInit, tprio_t prio);
void McuMotorTriggerFwUpdate(McuMotor* pMot, uint8_t force);
void McuMotorGet(McuMotor* pMot, McuMotorMeasurement* pMeas);

void McuMotorSetOff(McuMotor* pMot);
void McuMotorSetVoltageAB(McuMotor* pMot, float voltageA_V, float voltageB_V);
void McuMotorSetElectricalAngle(McuMotor* pMot, float angle_rad, float voltage_V);
void McuMotorSetVoltageDQ(McuMotor* pMot, float voltageD_V, float voltageQ_V);
void McuMotorSetCurrentDVoltageQ(McuMotor* pMot, float currentD_A, float voltageQ_V);
void McuMotorSetCurrentDQ(McuMotor* pMot, float currentD_A, float currentQ_A);
void McuMotorSetVelocity(McuMotor* pMot, float velocity_radDs, float currentD_A, float currentQ_A);

void McuMotorFocConfigDescInit(ConfigFileDesc* pDesc);
void McuMotorVelConfigDescInit(ConfigFileDesc* pDesc);

uint8_t McuMotorClampFocConfig(McuMotorFocConfig* pCfg, const McuMotorCtrlParams* pParams);
uint8_t McuMotorClampVelConfig(McuMotorVelConfig* pCfg, const McuMotorCtrlParams* pParams);

void McuMotorDebugRecord(McuMotor* pMot, uint8_t recordMode, uint8_t isOneShot);
void McuMotorDebugRecordStop(McuMotor* pMot);
