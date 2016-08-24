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

#include "traj_commander.h"
#include "crtp.h"
#include "configblock.h"
#include "param.h"



typedef struct
{
  struct TrajCommanderCrtpValues targetVal[2];
  bool activeSide;
  uint32_t timestamp;
} TrajCommanderCache;


static bool isInit;

static bool posPFMode = true;  //YHJ new Path Following Mode 
// true : Position Cmd
// false: Velocity Cmd

static TrajCommanderCache crtpCache;
static TrajCommanderCache extrxCache; // YHJ I don't think that we need this.
static TrajCommanderCache* activeCache;

static uint32_t lastUpdate;
static bool isInactive;

static void trajCommanderCrtpCB(CRTPPacket* pk);
static void commanderPFCacheSelectorUpdate(void); // YHJ I don't think that we need this.

/* Private functions */
static float trajCommanderGetActiveTgtX(void)
{
  return activeCache->targetVal[activeCache->activeSide].tgtX;
}

static float trajCommanderGetActiveTgtY(void)
{
  return activeCache->targetVal[activeCache->activeSide].tgtY;
}

static float trajCommanderGetActiveTgtZ(void)
{
  commanderPFCacheSelectorUpdate();
  return activeCache->targetVal[activeCache->activeSide].tgtZ;
}

static float trajCommanderGetActiveTgtYaw(void)
{
  return activeCache->targetVal[activeCache->activeSide].tgtYaw;
}


//YHJ I don't think that we need this.
static void commanderPFCacheSelectorUpdate(void)
{
  uint32_t tickNow = xTaskGetTickCount();

  // Check inputs and prioritize. Extrx higher then crtp
  if ((tickNow - extrxCache.timestamp) < TRAJ_COMMANDER_WDT_TIMEOUT_STABILIZE){
    activeCache = &extrxCache;
  } else if ((tickNow - crtpCache.timestamp) < TRAJ_COMMANDER_WDT_TIMEOUT_STABILIZE){
    activeCache = &crtpCache;
  } else if ((tickNow - extrxCache.timestamp) < TRAJ_COMMANDER_WDT_TIMEOUT_SHUTDOWN){
    activeCache = &extrxCache;
  } else if ((tickNow - crtpCache.timestamp) < TRAJ_COMMANDER_WDT_TIMEOUT_SHUTDOWN){
    activeCache = &crtpCache;
  } else {
    activeCache = &crtpCache;
  }
}

static void trajCommanderCrtpCB(CRTPPacket* pk)
{
  crtpCache.targetVal[!crtpCache.activeSide] = *((struct TrajCommanderCrtpValues*)pk->data);
  crtpCache.activeSide = !crtpCache.activeSide;
  crtpCache.timestamp = xTaskGetTickCount();
}

/* Public functions */
void trajCommanderInit(void)
{
  if(isInit)
    return;

  crtpInit();
  crtpRegisterPortCB(CRTP_PROT_TRAJ_CMD, trajCommanderCrtpCB);

  activeCache = &crtpCache;
  lastUpdate = xTaskGetTickCount();
  isInactive = true;
  isInit = true;
}

bool trajCommanderTest(void)
{
  crtpTest();
  return isInit;
}


void trajCommanderExtrxSet(const struct TrajCommanderCrtpValues* val)
{
  extrxCache.targetVal[!extrxCache.activeSide] = *((struct TrajCommanderCrtpValues*)val);
  extrxCache.activeSide = !extrxCache.activeSide;
  extrxCache.timestamp = xTaskGetTickCount();
}


uint32_t trajCommanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}


void commanderPFGetsetpoint(setpoint_t *setpoint, const state_t *state)
{
    // Thrust
    if (posPFMode){
      setpoint->thrust = 0;
      setpoint->mode.z = modeAbs;
      setpoint->position.z = trajCommanderGetActiveTgtZ();
    }
    else{
      setpoint->thrust = 0;
      setpoint->mode.z = modeVelocity;
      setpoint->velocity.z = trajCommanderGetActiveTgtZ();
    }

    // roll/pitch
    if (posPFMode){
      setpoint->mode.x = modeAbs;
      setpoint->mode.y = modeAbs;
      setpoint->mode.roll = modeDisable;
      setpoint->mode.pitch = modeDisable;

      setpoint->position.x = trajCommanderGetActiveTgtX(); 
      setpoint->position.y = trajCommanderGetActiveTgtY();
      setpoint->attitude.roll  = 0;
      setpoint->attitude.pitch = 0;
    }
    else{ //velocity Pathfollowing mode
      setpoint->mode.x = modeVelocity;
      setpoint->mode.y = modeVelocity;
      setpoint->mode.roll = modeDisable;
      setpoint->mode.pitch = modeDisable;

      setpoint->velocity.x = trajCommanderGetActiveTgtX();
      setpoint->velocity.y = trajCommanderGetActiveTgtY();
      setpoint->attitude.roll  = 0;
      setpoint->attitude.pitch = 0;
    }

    // yaw
    setpoint->attitudeRate.yaw  = trajCommanderGetActiveTgtYaw();
    setpoint->mode.yaw = modeAbs;
}

PARAM_GROUP_START(PFmode)
PARAM_ADD(PARAM_UINT8, posPFMode, &posPFMode) //YHJ
PARAM_GROUP_STOP(PFmode)
