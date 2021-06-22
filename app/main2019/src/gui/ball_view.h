/*
 * ball_view.h
 *
 *  Created on: 05.07.2019
 *      Author: SabolcJ
 */

#pragma once

#include "gfx.h"
#include "commands.h"

GHandle BallViewCreate();
void BallViewUpdateCamStats(const ExtCameraStats* pStats);
void BallViewUpdateDetections(const ExtBallDetections* pDetections);
