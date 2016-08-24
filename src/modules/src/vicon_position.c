/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  //YHJ  This is to receive Vicon Data from Crazyflie-ROS.
 *  //This module is written using the Commander.o . 
 */


#include "FreeRTOS.h"
#include "task.h"

#include "vicon_position.h"
#include "crtp.h"
#include "configblock.h"



typedef struct
{
  struct ViconPositionCrtpValues targetVal[2];
  bool activeSide;
  uint32_t timestamp;
} ViconPositionCache;


static bool isInit;
static ViconPositionCache crtpCache;
// static ViconPositionCache extrxCache; //YHJ <- I don't think that we need this.
static ViconPositionCache* activeCache;

static uint32_t lastUpdate;
static bool isInactive;

static void viconPositionCrtpCB(CRTPPacket* pk);
//static void viconPositionCacheSelectorUpdate(void); //YHJ <- I don't think that we need this.

/* Private functions */
static float viconPositionGetActiveViconX(void)
{
  return activeCache->targetVal[activeCache->activeSide].viconX;
}

static float viconPositionGetActiveViconY(void)
{
  return activeCache->targetVal[activeCache->activeSide].viconY;
}

static float viconPositionGetActiveViconZ(void)
{
  return activeCache->targetVal[activeCache->activeSide].viconZ;
}

static float viconPositionGetActiveViconYaw(void)
{
  return activeCache->targetVal[activeCache->activeSide].viconYaw;
}

/*
//YHJ I don't think we need the below
static void viconPositionCacheSelectorUpdate(void)
{
  uint32_t tickNow = xTaskGetTickCount();

  // Check inputs and prioritize. Extrx higher then crtp 
  if ((tickNow - extrxCache.timestamp) < VICON_POSITION_WDT_TIMEOUT_STABILIZE){
    activeCache = &extrxCache;
  } else if ((tickNow - crtpCache.timestamp) < VICON_POSITION_WDT_TIMEOUT_STABILIZE){
    activeCache = &crtpCache;
  } else if ((tickNow - extrxCache.timestamp) < VICON_POSITION_WDT_TIMEOUT_SHUTDOWN){
    activeCache = &extrxCache;
  } else if ((tickNow - crtpCache.timestamp) < VICON_POSITION_WDT_TIMEOUT_SHUTDOWN){
    activeCache = &crtpCache;
  } else {
    activeCache = &crtpCache;
  }
}
*/
static void viconPositionCrtpCB(CRTPPacket* pk)
{
  crtpCache.targetVal[!crtpCache.activeSide] = *((struct ViconPositionCrtpValues*)pk->data);
  crtpCache.activeSide = !crtpCache.activeSide;
  crtpCache.timestamp = xTaskGetTickCount();
}

/* Public functions */
void viconPositionInit(void)
{
  if(isInit)
    return;

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_VICON, viconPositionCrtpCB);

  activeCache = &crtpCache;
  lastUpdate = xTaskGetTickCount();
  isInactive = true;
  isInit = true;
}

bool viconPositionTest(void)
{
  crtpTest();
  return isInit;
}

/* //YHJ I don't think we need the below
void viconPositionExtrxSet(const struct ViconPositionCrtpValues* val)
{
  extrxCache.targetVal[!extrxCache.activeSide] = *((struct ViconPositionCrtpValues*)val);
  extrxCache.activeSide = !extrxCache.activeSide;
  extrxCache.timestamp = xTaskGetTickCount();
}
*/

uint32_t viconPositionGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

void viconPositionGetXYZ(float* viconX, float* viconY, float* viconZ, float* viconYaw)
{
  *viconX  = viconPositionGetActiveViconX();
  *viconY  = viconPositionGetActiveViconY();
  *viconZ  = viconPositionGetActiveViconZ();
  *viconYaw  = viconPositionGetActiveViconYaw();
}
