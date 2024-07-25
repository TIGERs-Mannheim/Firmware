#include "robot_specs.h"
#include "struct_ids.h"

ConfigFileDesc robotSpecsConfigDescPhysical =
	{ SID_CFG_CTRL_PHYSICAL_PARAMS, 0, "ctrl/physical", 11, (ElementDesc[]) {
		{ FLOAT, "wheel_radius", "m", "Wheel Radius" },
		{ FLOAT, "front_angle", "deg", "Front Angle" },
		{ FLOAT, "back_angle", "deg", "Back  Angle" },
		{ FLOAT, "bot_radius", "m", "Bot Radius" },
		{ FLOAT, "mass", "kg", "Mass" },
		{ FLOAT, "dribbler_distance", "m", "Robot Center to Ball Center" },
		{ FLOAT, "dribbler_width", "m", "Ball movement range at dribbler" },
		{ FLOAT, "cog_x", "m", "X center of gravity offset" },
		{ FLOAT, "cog_y", "m", "Y center of gravity offset" },
		{ FLOAT, "cog_z", "m", "Z center of gravity offset" },
		{ FLOAT, "mass_distri_z", "-", "Factor for mass distribution around z axis" },
	} };

ConfigFileDesc robotSpecsConfigDescDriveTrain =
	{ SID_CFG_CTRL_DRIVE_TRAIN, 0, "ctrl/drive_train", 9, (ElementDesc[]) {
		{ FLOAT, "motor2wheel", "-", "Motor to Wheel Ratio" },
		{ FLOAT, "wheel2motor", "-", "Wheel to Motor Ratio" },
		{ FLOAT, "motor_km", "Nm/A", "Motor Torque Constant" },
		{ FLOAT, "motor_kn", "rad/(s*V)", "Motor Speed Constant" },
		{ FLOAT, "motor_ke", "V/rad", "Motor back-EMF constant" },
		{ FLOAT, "motor_kf", "Nm*s", "Motor Friction Constant" },
		{ FLOAT, "motor_r", "Ohm", "Motor Resistance" },
		{ FLOAT, "motor_l", "H", "Motor Inductance" },
		{ FLOAT, "motor_pp", "-", "Motor Pole Pairs" },
	} };

ConfigFileDesc robotSpecsConfigDescDribbler =
	{ SID_CFG_CTRL_DRIBBLER, 0, "ctrl/dribbler", 10, (ElementDesc[]) {
		{ FLOAT, "motor2bar", "-", "Motor to Bar Ratio" },
		{ FLOAT, "bar2motor", "-", "Bar to Motor Ratio" },
		{ FLOAT, "bar_diameter", "m", "Dribbling bar diameter" },
		{ FLOAT, "motor_km", "Nm/A", "Motor Torque Constant" },
		{ FLOAT, "motor_kn", "rad/(s*V)", "Motor Speed Constant" },
		{ FLOAT, "motor_ke", "V/rad", "Motor back-EMF constant" },
		{ FLOAT, "motor_kf", "Nm*s", "Motor Friction Constant" },
		{ FLOAT, "motor_r", "Ohm", "Motor Resistance" },
		{ FLOAT, "motor_l", "H", "Motor Inductance" },
		{ FLOAT, "motor_pp", "-", "Motor Pole Pairs" },
	} };
