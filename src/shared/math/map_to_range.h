/*
 * map_to_range.h
 *
 *  Created on: 11.12.2017
 *      Author: AndreR
 */

#pragma once

/**
 * Map an input value to a clamped range.
 *
 * - If value is below inMin, the output is outMin
 * - If value is above inMax, the output is outMax
 * - If value is between inMin and inMax, its relative value is mapped to the range outMin to outMax
 *
 * @param inMin Minimum input value
 * @param inMax Maximum input value
 * @param outMin Mapped minimum output value
 * @param outMax Mapped maximum output value
 * @param value Input value
 * @return Clamped and mapped value
 */
float MapToRangef32(float inMin, float inMax, float outMin, float outMax, float value);
