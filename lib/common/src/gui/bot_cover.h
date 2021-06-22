/*
 * bot_cover.h
 *
 *  Created on: 05.10.2015
 *      Author: AndreR
 */

#ifndef BOT_COVER_H_
#define BOT_COVER_H_

#include "gfx.h"

GHandle	BotCoverCreate(coord_t x, coord_t y, GHandle hParent, uint32_t id, GButtonObject* pInstance);
void	BotCoverSetId(GHandle hCover, uint32_t id);

#endif /* BOT_COVER_H_ */
