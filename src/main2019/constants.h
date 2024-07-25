#pragma once

#include "ch.h"

/////////////////////////////////////////////////
// Task Priorities
/////////////////////////////////////////////////
#define TASK_PRIO_KICKER		(NORMALPRIO + 40)
#define TASK_PRIO_ADC_KICKER	(NORMALPRIO + 30)
#define TASK_PRIO_IMU			(NORMALPRIO + 20)
#define TASK_PRIO_MAG			(NORMALPRIO + 20)
#define TASK_PRIO_TOUCH			(NORMALPRIO + 20)
#define TASK_PRIO_MCU_MOTOR		(NORMALPRIO + 15)
#define TASK_PRIO_MCU_DRIBBLER	(NORMALPRIO + 10)
#define TASK_PRIO_PORT_EX		(NORMALPRIO -  1)
#define TASK_PRIO_PWR_INA226	(NORMALPRIO -  1)
#define TASK_PRIO_POWER_CONTROL	(NORMALPRIO -  1)
#define TASK_PRIO_TIGER_BOT		(NORMALPRIO -  2)
#define TASK_PRIO_RPI_SERIAL	(NORMALPRIO -  3)
#define TASK_PRIO_ROBOT_PI		(NORMALPRIO -  4)
#define TASK_PRIO_MPU_EXT		(NORMALPRIO -  5)
#define TASK_PRIO_SHELL			(NORMALPRIO -  5)
#define TASK_PRIO_CLI_SERIAL	(NORMALPRIO - 10)
#define TASK_PRIO_PATTERN_IDENT	(NORMALPRIO - 15)
#define TASK_PRIO_TESTS			(NORMALPRIO - 50)

/////////////////////////////////////////////////
// IRQ Levels
/////////////////////////////////////////////////

#define IRQL_WIFI_HIGH_PRIO	0
#define IRQL_WIFI_SPI_DONE	1
#define IRQL_WIFI_PIN		2
#define IRQL_WIFI_TIMEOUT	3

#define IRQL_WIFI_LOW_PRIO	5

#define IRQL_KICK_CTRL		6
#define IRQL_IR_COMM		8
#define IRQL_MOTOR_COMMS	8
#define IRQL_SPI_KICKER		8
#define IRQL_SPI4			9
#define IRQL_USB			10
#define IRQL_SDMMC1			10
#define IRQL_RPI_SERIAL		12
#define IRQL_SHELL_SERIAL	14
#define IRQL_EXTI3			15	// KILL
#define IRQL_I2C2			15
#define IRQL_I2C_SOFT		15
