#pragma once

#include "gfx.h"
#include "ext_commands.h"

GHandle BallViewCreate();
void BallViewUpdateCamStats(const ExtCameraStats* pStats);
void BallViewUpdateDetections(const ExtBallDetections* pDetections);
