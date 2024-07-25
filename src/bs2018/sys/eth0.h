#pragma once

#include "ch.h"
#include "util/shell_cmd.h"
#include "hal/eth.h"

extern Eth eth0;

void Eth0Init(tprio_t prio, uint32_t irqLevelMii);
