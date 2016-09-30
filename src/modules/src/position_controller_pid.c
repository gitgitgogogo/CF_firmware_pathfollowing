/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 * position_estimator_pid.c: PID-based implementation of the position controller
 */

#include <math.h>
#include "num.h"

#include "commander.h"
#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"

struct pidInit_s {
  float kp;
  float ki;
  float kd;
};

struct pidAxis_s {
  PidObject pid;

  struct pidInit_s init;
  mode_t previousMode;
  float setpoint;

  float output;
};

struct this_s {
  struct pidAxis_s pidX;
  struct pidAxis_s pidY;
  struct pidAxis_s pidZ;

  uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
};

// Maximum roll/pitch angle permited
static float rpLimit = 15;

#define DT 0.01

#ifndef UNIT_TEST
static struct this_s this = {

  .pidX = {
    .init = {
      .kp = 10.0, //25,
      .ki =  0.2,  //0.28,
      .kd = 10.0// 10.0 //7 
    }
  },

  .pidY = {
    .init = {
      .kp = 10.0, //25,
      .ki = 0.2, //0.28,
      .kd = 10.0 //10.0//7
    }
  },

  .pidZ = {
    .init = {
      .kp = 10000.0, //20000.0,
      .ki = 0.0,      // 3000.0,
      .kd = 5000.0   //15000.0 // 15000.0
    }
  },

  .thrustBase = 23000,  //45000,//36000, <-Original YHJ 
};
#endif

static float runPid(float input, struct pidAxis_s *axis, mode_t mode,
                    float setpointPos, float setpointVel, float dt) {
  if (axis->previousMode == modeDisable && mode != modeDisable) {
    if (mode == modeVelocity) {
      axis->setpoint = input;
    } else {
      axis->setpoint = setpointPos;
    }
    pidInit(&axis->pid, axis->setpoint, axis->init.kp, axis->init.ki, axis->init.kd, dt);
  }
  axis->previousMode = mode;

  // This is a position controller so if the setpoint is in velocity we
  // integrate it to get a position setpoint
  if (mode == modeAbs) {
    axis->setpoint = setpointPos;
  } else if (mode == modeVelocity) {
    //axis->setpoint = setpointVel; //YHJ
    axis->setpoint += setpointVel * dt; //Original <- YHJ Commneted Out 
  }

  pidSetDesired(&axis->pid, axis->setpoint);
  return pidUpdate(&axis->pid, input, true);
}

void positionController(float* thrust, attitude_t *attitude, const state_t *state,
                                                             const setpoint_t *setpoint)
{
  // X, Y
  /*
  float x_cntl_deriv = (0.0 - state->velocity.x)*10.0;
  float y_cntl_deriv = (0.0 - state->velocity.y)*10.0;
  float z_cntl_deriv = (0.0 - state->velocity.z)*15000.0;
  */
  float x = runPid(state->position.x, &this.pidX, setpoint->mode.x, setpoint->position.x, setpoint->velocity.x, DT); //+ x_cntl_deriv;
  float y = runPid(state->position.y, &this.pidY, setpoint->mode.y, setpoint->position.y, setpoint->velocity.y, DT); //+ y_cntl_deriv;

  float yawRad = state->attitude.yaw * (float)M_PI / 180;
  attitude->pitch = - (x * cosf(yawRad)) - (y * sinf(yawRad));
  attitude->roll =  - (y * cosf(yawRad)) + (x * sinf(yawRad));

  attitude->roll = max(min(attitude->roll, rpLimit), -rpLimit);
  attitude->pitch = max(min(attitude->pitch, rpLimit), -rpLimit);

  // Z
  float newThrust = runPid(state->position.z, &this.pidZ, setpoint->mode.z, setpoint->position.z, setpoint->velocity.z, DT); // + z_cntl_deriv;
  *thrust = newThrust + this.thrustBase;
}


LOG_GROUP_START(posCtl)
LOG_ADD(LOG_FLOAT, targetX, &this.pidX.setpoint)
LOG_ADD(LOG_FLOAT, targetY, &this.pidY.setpoint)
LOG_ADD(LOG_FLOAT, targetZ, &this.pidZ.setpoint)
//LOG_ADD(LOG_FLOAT, cntlX, &this.pidX.pid.outTotal)
//LOG_ADD(LOG_FLOAT, cntlY, &this.pidY.pid.outTotal)
//LOG_ADD(LOG_FLOAT, cntlZ, &this.pidZ.pid.outTotal)
LOG_GROUP_STOP(posCtl)

LOG_GROUP_START(posCtl_Pid)
LOG_ADD(LOG_FLOAT, x_I, &this.pidX.pid.outI)
LOG_ADD(LOG_FLOAT, x_D, &this.pidX.pid.outD)
LOG_ADD(LOG_FLOAT, y_I, &this.pidY.pid.outI)
LOG_ADD(LOG_FLOAT, y_D, &this.pidY.pid.outD)
LOG_ADD(LOG_FLOAT, z_I, &this.pidZ.pid.outI)
LOG_ADD(LOG_FLOAT, z_D, &this.pidZ.pid.outD)
LOG_GROUP_STOP(posCtl_Pid)



PARAM_GROUP_START(posCtlPid)

PARAM_ADD(PARAM_FLOAT, xKp, &this.pidX.init.kp)
PARAM_ADD(PARAM_FLOAT, xKi, &this.pidX.init.ki)
PARAM_ADD(PARAM_FLOAT, xKd, &this.pidX.init.kd)

PARAM_ADD(PARAM_FLOAT, yKp, &this.pidY.init.kp)
PARAM_ADD(PARAM_FLOAT, yKi, &this.pidY.init.ki)
PARAM_ADD(PARAM_FLOAT, yKd, &this.pidY.init.kd)

PARAM_ADD(PARAM_FLOAT, zKp, &this.pidZ.init.kp)
PARAM_ADD(PARAM_FLOAT, zKi, &this.pidZ.init.ki)
PARAM_ADD(PARAM_FLOAT, zKd, &this.pidZ.init.kd)

PARAM_ADD(PARAM_UINT16, thrustBase, &this.thrustBase)

PARAM_ADD(PARAM_FLOAT, rpLimit, &rpLimit)
PARAM_GROUP_STOP(posCtlPid)
