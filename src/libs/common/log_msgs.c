/*
 * log_msgs.c
 *
 *  Created on: 29.02.2016
 *      Author: AndreR
 */

#include "log_msgs.h"
#include "struct_ids.h"

const LogMessageDesc logMessageDescriptions[10] = {

	{ SID_GYRO_RAW, "gyro_raw", 3, (ElementDesc[]) {
		{ INT16, "X", "raw", "X axis" },
		{ INT16, "Y", "raw", "Y axis" },
		{ INT16, "Z", "raw", "Z axis" },
	}, },
	{ SID_ACC_RAW, "acc_raw", 3, (ElementDesc[]) {
		{ INT16, "X", "raw", "X axis" },
		{ INT16, "Y", "raw", "Y axis" },
		{ INT16, "Z", "raw", "Z axis" },
	}, },
	{ SID_SENSORS, "sensors", 75, (ElementDesc[]) {
		// 0 + 5
		{ UINT32,	"vel_t", "us",	"Motor velocity timestamp" },
		{ FLOAT,	"vel_m1", "rad/s","Motor 1 angular velocity" },
		{ FLOAT,	"vel_m2", "rad/s","Motor 2 angular velocity" },
		{ FLOAT,	"vel_m3", "rad/s","Motor 3 angular velocity" },
		{ FLOAT,	"vel_m4", "rad/s","Motor 4 angular velocity" },

		// 5 + 5
		{ UINT32,	"gyr_updated", "bool", "Gyroscope data updated" },
		{ UINT32,	"gyr_t", "us", "Timestamp" },
		{ FLOAT,	"gyr_x", "rad/s", "Angular velocity around X axis" },
		{ FLOAT,	"gyr_y", "rad/s", "Angular velocity around Y axis" },
		{ FLOAT,	"gyr_z", "rad/s", "Angular velocity around Z axis" },

		// 10 + 5
		{ UINT32,	"acc_updated", "bool", "Accelerometer data updated" },
		{ UINT32,	"acc_t", "us", "Timestamp" },
		{ FLOAT,	"acc_x", "m/s^2", "Acceleration along X axis" },
		{ FLOAT,	"acc_y", "m/s^2", "Acceleration along Y axis" },
		{ FLOAT,	"acc_z", "m/s^2", "Acceleration along Z axis" },

		// 15 + 8
		{ UINT32,	"pos_updated", "bool", "Position data updated" },
		{ UINT32,	"pos_t", "us", "Timestamp" },
		{ UINT32,	"pos_delay", "us", "Vision Delay" },
		{ FLOAT,	"pos_x", "m", "X Position" },
		{ FLOAT,	"pos_y", "m", "Y Position" },
		{ FLOAT,	"pos_z", "m", "Z Position" },
		{ UINT32,	"cam_id", "-", "Camera ID" },
		{ UINT32,	"no_vision", "-", "No Vision Flag" },

		// 23 + 4
		{ FLOAT,	"supply_bat", "V", "Battery voltage" },
		{ FLOAT,	"supply_cur", "A", "Current consumption" },
		{ UINT32,	"bat_empty", "bool", "Battery empty" },
		{ UINT32,	"exhausted", "bool", "Battery low" },

		// 27 + 5
		{ UINT32,	"vol_t", "us", "Motor voltage timestamp" },
		{ FLOAT,	"vol_m1", "V", "Motor 1 voltage" },
		{ FLOAT,	"vol_m2", "V", "Motor 2 voltage" },
		{ FLOAT,	"vol_m3", "V", "Motor 3 voltage" },
		{ FLOAT,	"vol_m4", "V", "Motor 4 voltage" },

		// 32 + 4
		{ FLOAT,	"kicker_level", "V", "Kicker Level" },
		{ FLOAT,	"kicker_chg", "A", "Kicker charge current" },
		{ UINT32,	"kicker_counter", "-",	"Kick Counter" },
		{ UINT32,	"kicker_flags", "-",	"Status flags" },

		// 36 + 7
		{ FLOAT,	"dribbler_hall_speed", "rad/s", "Dribbler hall speed" },
		{ FLOAT,	"dribbler_aux_speed", "rad/s", "Dribbler aux. speed" },
		{ FLOAT,	"dribbler_temp", "C", "Dribbler temperature" },
		{ FLOAT,	"dribbler_voltage", "V", "Dribbler voltage" },
		{ FLOAT,	"dribbler_current", "A", "Dribbler current" },
		{ UINT32,	"dribbler_overheated", "bool",	"Dribbler overheated" },
		{ UINT32,	"dribbler_hall", "-",	"Dribbler hall position" },

		// 43 + 6
		{ FLOAT,	"ir_on", "V", "IR Voltage Sender On" },
		{ FLOAT,	"ir_off", "V", "IR Voltage Sender Off" },
		{ UINT32,	"ir_interrupted", "bool", "Barrier interrupted" },
		{ UINT32,	"ir_ballDetected", "bool", "Ball detected in front of IR array" },
		{ FLOAT,	"ir_ball_est_x", "mm", "Estimated x position of the ball" },
		{ FLOAT,	"ir_ball_est_y", "mm", "Estimated y position of the ball" },

		// 49 + 5
		{ UINT32,	"cur_t", "us", "Motor current timestamp" },
		{ FLOAT,	"cur_m1", "A", "Motor 1 current" },
		{ FLOAT,	"cur_m2", "A", "Motor 2 current" },
		{ FLOAT,	"cur_m3", "A", "Motor 3 current" },
		{ FLOAT,	"cur_m4", "A", "Motor 4 current" },

		// 54 + 13
		{ UINT32,	"ball_updated", "-", "Ball detection updated" },
		{ UINT32,	"ball_t", "us", "Ball camera frame timestamp" },
		{ FLOAT,	"ball_pos_x", "m", "Ball position X (map frame)" },
		{ FLOAT,	"ball_pos_y", "m", "Ball position Y (map frame)" },
		{ FLOAT,	"ball_pos_z", "m", "Ball position Z (map frame)" },
		{ FLOAT,	"ball_vel_x", "m/s", "Ball velocity X (map frame)" },
		{ FLOAT,	"ball_vel_y", "m/s", "Ball velocity Y (map frame)" },
		{ FLOAT,	"ball_vel_z", "m/s", "Ball velocity Z (map frame)" },
		{ FLOAT,	"ball_line_pos_x", "m", "Ball line fit position X" },
		{ FLOAT,	"ball_line_pos_y", "m", "Ball line fit position Y" },
		{ FLOAT,	"ball_line_vel_x", "m/s", "Ball line fit velocity X" },
		{ FLOAT,	"ball_line_vel_y", "m/s", "Ball line fit velocity Y" },
		{ UINT32,	"ball_tracker_id", "-", "Ball tracker id" },

		// 67 + 6
		{ UINT32,	"mag_updated", "bool", "Magnetic data updated" },
		{ UINT32,	"mag_t", "us", "Timestamp" },
		{ FLOAT,	"mag_x", "mT", "Magnetic axis 1" },
		{ FLOAT,	"mag_y", "mT", "Magnetic axis 2" },
		{ FLOAT,	"mag_z", "mT", "Magnetic axis 3" },
		{ FLOAT,	"mag_temp", "C", "Magnetic sensor temperature" },

		// 73 + 2
		{ UINT32,	"pattern_id", "ID", "Detected pattern ID" },
		{ UINT32,	"pattern_flags", "", "Pattern ident status" },
	}, },
	{ SID_CTRL_STATE, "ctrl_state", 18, (ElementDesc[]) {
		{ FLOAT,	"pos_x", "m", "X Position" },
		{ FLOAT,	"pos_y", "m", "Y Position" },
		{ FLOAT,	"pos_z", "rad", "Z Position" },

		{ FLOAT,	"vel_x", "m/s", "X Velocity" },
		{ FLOAT,	"vel_y", "m/s", "Y Velocity" },
		{ FLOAT,	"vel_z", "rad/s", "Z Velocity" },

		{ FLOAT,	"acc_x", "m/s^2", "X Acceleration" },
		{ FLOAT,	"acc_y", "m/s^2", "Y Acceleration" },
		{ FLOAT,	"acc_z", "rad/s^2", "Z Acceleration" },

		{ FLOAT,	"mag_z", "rad", "Z Position (mag)"},

		{ UINT32,	"pos_updated", "bool", "Position estimation updated" },
		{ UINT32,	"vel_updated", "bool", "Velocity estimation updated" },
		{ UINT32,	"acc_updated", "bool", "Acceleration estimation updated" },

		{ FLOAT,	"dribbler_vel", "rad/s", "Dribbler velocity"},
		{ FLOAT,	"dribbler_cur", "A", "Dribbler current"},

		{ UINT32,	"ball_ir_statte", "enum", "Ball IR detection state" },
		{ FLOAT,	"ball_ir_pos_x", "mm", "Ball Position from IR Array X"},
		{ FLOAT,	"ball_ir_pos_y", "mm", "Ball Position from IR Array Y"},
	}, },
	{ SID_CTRL_OUTPUT, "ctrl_output", 17, (ElementDesc[]) {
		{ FLOAT,	"vel_m1", "rad/s", "Motor 1 velocity" },
		{ FLOAT,	"vel_m2", "rad/s", "Motor 2 velocity" },
		{ FLOAT,	"vel_m3", "rad/s", "Motor 3 velocity" },
		{ FLOAT,	"vel_m4", "rad/s", "Motor 4 velocity" },
		{ FLOAT,	"vol_m1", "V", "Motor 1 voltage" },
		{ FLOAT,	"vol_m2", "V", "Motor 2 voltage" },
		{ FLOAT,	"vol_m3", "V", "Motor 3 voltage" },
		{ FLOAT,	"vol_m4", "V", "Motor 4 voltage" },
		{ FLOAT,	"acc_m1", "rad/s^2", "Motor 1 acceleration" },
		{ FLOAT,	"acc_m2", "rad/s^2", "Motor 2 acceleration" },
		{ FLOAT,	"acc_m3", "rad/s^2", "Motor 3 acceleration" },
		{ FLOAT,	"acc_m4", "rad/s^2", "Motor 4 acceleration" },
		{ UINT32,	"ctrlMode", "-", "Motor controller mode"},
		{ FLOAT,	"torque_m1", "Nm", "Motor 1 torque" },
		{ FLOAT,	"torque_m2", "Nm", "Motor 2 torque" },
		{ FLOAT,	"torque_m3", "Nm", "Motor 3 torque" },
		{ FLOAT,	"torque_m4", "Nm", "Motor 4 torque" },
	}, },
	{ SID_ROBOT_PERFORMANCE, "performance", 6, (ElementDesc[]) {
		{ UINT32,	"input_time", "us", "Input fetching time" },
		{ UINT32,	"estimation_time", "us", "Sensor fusion processing time" },
		{ UINT32,	"skill_time", "us", "Skill execution time" },
		{ UINT32,	"control_time", "us", "Control processing time" },
		{ UINT32,	"output_time", "us", "Data output time" },
		{ UINT32,	"misc_time", "us", "Book keeping time" },
	}, },
	{ SID_CTRL_REF, "ctrl_ref", 9, (ElementDesc[]) {
		{ FLOAT,	"traj_pos_x", "m", "Global Position from Trajectory X" },
		{ FLOAT,	"traj_pos_y", "m", "Global Position from Trajectory Y"},
		{ FLOAT,	"traj_pos_z", "m", "Global Position from Trajectory Z" },

		{ FLOAT,	"traj_vel_local_x", "m/s", "Local Velocity from Trajectory X" },
		{ FLOAT,	"traj_vel_local_y", "m/s", "Local Velocity from Trajectory Y"},
		{ FLOAT,	"traj_vel_local_z", "m/s", "Local Velocity from Trajectory Z" },

		{ FLOAT,	"traj_acc_local_x", "m/s^2", "Local Acceleration from Trajectory X" },
		{ FLOAT,	"traj_acc_local_y", "m/s^2", "Local Acceleration from Trajectory Y" },
		{ FLOAT,	"traj_acc_local_z", "m/s^2", "Local Acceleration from Trajectory Z" },
	}, },
	{ SID_SKILL_OUTPUT, "skill_output", 28, (ElementDesc[]) {
		{ FLOAT,	"pos_x", "m", "X Position" },
		{ FLOAT,	"pos_y", "m", "Y Position" },
		{ FLOAT,	"pos_z", "rad", "Z Position" },

		{ FLOAT,	"vel_x", "m/s", "X Velocity Local" },
		{ FLOAT,	"vel_y", "m/s", "Y Velocity Local" },
		{ FLOAT,	"vel_z", "rad/s", "Z Velocity Local" },

		{ FLOAT,	"vel_w1", "rad/s", "Wheel 1 velocity" },
		{ FLOAT,	"vel_w2", "rad/s", "Wheel 2 velocity" },
		{ FLOAT,	"vel_w3", "rad/s", "Wheel 3 velocity" },
		{ FLOAT,	"vel_w4", "rad/s", "Wheel 4 velocity" },

		{ FLOAT,	"force_x", "N", "X Force Local" },
		{ FLOAT,	"force_y", "N", "Y Force Local" },
		{ FLOAT,	"torque_z", "Nm", "Z Torque Local" },

		{ UINT32,	"mode_xy", "-", "XY control mode" },
		{ UINT32,	"mode_w", "-", "W control mode" },

		{ FLOAT,	"prime_dir", "rad", "Primary Direction" },

		{ FLOAT,	"vel_max_xy", "m/s", "Max. velocity XY" },
		{ FLOAT,	"acc_max_xy", "m/s^2", "Max. acceleration XY" },
		{ FLOAT,	"vel_max_w", "rad/s", "Max. velocity W" },
		{ FLOAT,	"acc_max_w", "rad/s^2", "Max. acceleration W" },
		{ UINT32,	"strict_vel", "bool", "Strict velocity limit" },

		{ FLOAT,	"dribbler_speed", "RPM", "Dribbler RPM" },
		{ FLOAT,	"dribbler_max_current", "A", "Dribbler Max. Current" },
		{ FLOAT,	"dribbler_voltage", "V", "Dribbler Voltage" },
		{ UINT32,	"dribbler_mode", "-", "Dribbler control mode" },

		{ UINT32,	"kicker_mode", "-", "Kicker control mode" },
		{ UINT32,	"kicker_device", "-", "Kicker device" },
		{ FLOAT,	"kicker_speed", "m/s", "Kick speed" },
	}, },
	{ SID_NETSTATS, "net_stats", 9, (ElementDesc[]) {
		{ FLOAT,	"rx_period", "s", "Time between two receptions" },
		{ FLOAT,	"rssi", "dBm", "Receive Signal Strength" },
		{ UINT16,	"tx_packets", "-", "Transmitted packets" },
		{ UINT16,	"tx_bytes", "-", "Transmitted bytes" },
		{ UINT16,	"rx_packets", "-", "Successfully received packets" },
		{ UINT16,	"rx_bytes", "-", "Successfully received bytes" },
		{ UINT16,	"rx_packets_lost", "-", "Lost received packets" },
		{ UINT16,	"tx_hl_packets_lost", "-", "Lost TX packets due to no mem" },
		{ UINT16,	"rx_hl_packets_lost", "-", "Lost RX packets due to no mem" },
	}, },
	{ SID_IR_DATA, "ir_data", 4 * 5 + 3, (ElementDesc[]) {
		{ FLOAT,	"recv_1_1", "V", "Reciever 1_1" },
		{ FLOAT,	"recv_1_2", "V", "Reciever 1_2" },
		{ FLOAT,	"recv_1_3", "V", "Reciever 1_3" },
		{ FLOAT,	"recv_1_4", "V", "Reciever 1_4" },

		{ FLOAT,	"recv_2_1", "V", "Reciever 2_1" },
		{ FLOAT,	"recv_2_2", "V", "Reciever 2_2" },
		{ FLOAT,	"recv_2_3", "V", "Reciever 2_3" },
		{ FLOAT,	"recv_2_4", "V", "Reciever 2_4" },

		{ FLOAT,	"recv_3_1", "V", "Reciever 3_1" },
		{ FLOAT,	"recv_3_2", "V", "Reciever 3_2" },
		{ FLOAT,	"recv_3_3", "V", "Reciever 3_3" },
		{ FLOAT,	"recv_3_4", "V", "Reciever 3_4" },

		{ FLOAT,	"recv_4_1", "V", "Reciever 4_1" },
		{ FLOAT,	"recv_4_2", "V", "Reciever 4_2" },
		{ FLOAT,	"recv_4_3", "V", "Reciever 4_3" },
		{ FLOAT,	"recv_4_4", "V", "Reciever 4_4" },

		{ FLOAT,	"recv_5_1", "V", "Reciever 5_1" },
		{ FLOAT,	"recv_5_2", "V", "Reciever 5_2" },
		{ FLOAT,	"recv_5_3", "V", "Reciever 5_3" },
		{ FLOAT,	"recv_5_4", "V", "Reciever 5_4" },

		{ FLOAT,	"est_ball_x", "cm", "Estimated Ball Position (X)" },
		{ FLOAT,	"est_ball_y", "cm", "Estimated Ball Position (Y)" },

		{ UINT8,	"ball_detected", "bool", "Ball detected" },

	}, },
};
