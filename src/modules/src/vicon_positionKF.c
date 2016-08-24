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
 * //YHJ  This is to receive Kalman Filterd Vicon Data from Crazyflie-ROS.
 * //This module is written using the Commander.o .
 */


#include "FreeRTOS.h"
#include "task.h"

#include "vicon_positionKF.h"
#include "crtp.h"
#include "configblock.h"



typedef struct
{
  struct ViconPositionKFCrtpValues targetVal[2];
  bool activeSide;
  uint32_t timestamp;
} ViconPositionKFCache;


static bool isInit;
static ViconPositionKFCache crtpCache;
//static ViconPositionKFCache extrxCache; // YHJ I don't think that we need this.
static ViconPositionKFCache* activeCache;

static uint32_t lastUpdate;
static bool isInactive;

static void viconPositionKFCrtpCB(CRTPPacket* pk);
//static void viconPositionKFCachSelectorUpdate(void); // YHJ I don't think that we need this.

/* Private functions */
static float viconPositionKFGetActiveViconX(void)
{
  return activeCache->targetVal[activeCache->activeSide].viconX;
}

static float viconPositionKFGetActiveViconY(void)
{
  return activeCache->targetVal[activeCache->activeSide].viconY;
}

static float viconPositionKFGetActiveViconZ(void)
{
  return activeCache->targetVal[activeCache->activeSide].viconZ;
}

static float viconPositionKFGetActiveViconVx(void)
{
  return activeCache->targetVal[activeCache->activeSide].viconVx;
}

static float viconPositionKFGetActiveViconVy(void)
{
  return activeCache->targetVal[activeCache->activeSide].viconVy;
}

static float viconPositionKFGetActiveViconVz(void)
{
  return activeCache->targetVal[activeCache->activeSide].viconVz;
}

/* //YHJ I don't think that we need this.
static void viconPositionKFCacheSelectorUpdate(void)
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

static void viconPositionKFCrtpCB(CRTPPacket* pk)
{
  crtpCache.targetVal[!crtpCache.activeSide] = *((struct ViconPositionKFCrtpValues*)pk->data);
  crtpCache.activeSide = !crtpCache.activeSide;
  crtpCache.timestamp = xTaskGetTickCount();
}

/* Public functions */
void viconPositionKFInit(void)
{
  if(isInit)
    return;

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_VICON_KF, viconPositionKFCrtpCB);

  activeCache = &crtpCache;
  lastUpdate = xTaskGetTickCount();
  isInactive = true;
  isInit = true;
}

bool viconPositionKFTest(void)
{
  crtpTest();
  return isInit;
}

/* //YHJ I don't think we need this.
void viconPositionKFExtrxSet(const struct ViconPositionKFCrtpValues* val)
{
  extrxCache.targetVal[!extrxCache.activeSide] = *((struct ViconPositionKFCrtpValues*)val);
  extrxCache.activeSide = !extrxCache.activeSide;
  extrxCache.timestamp = xTaskGetTickCount();
}
*/

uint32_t viconPositionKFGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

void viconPositionKFGetXYZ(float* viconX, float* viconY, float* viconZ, float* viconVx, float* viconVy, float* viconVz)
{
  *viconX  = viconPositionKFGetActiveViconX();
  *viconY  = viconPositionKFGetActiveViconY();
  *viconZ  = viconPositionKFGetActiveViconZ();
  *viconVx  = viconPositionKFGetActiveViconVx();
  *viconVy  = viconPositionKFGetActiveViconVy();
  *viconVz  = viconPositionKFGetActiveViconVz();
}
