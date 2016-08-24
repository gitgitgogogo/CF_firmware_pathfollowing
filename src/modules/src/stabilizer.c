/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "estimator.h"
#include "commander.h"
#include "traj_commander.h"  //YHJ
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

static bool isInit;

static bool pathFollow_mode = false;  //YHJ


// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static void stabilizerTask(void* param);

void stabilizerInit(void)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit();
  stateControllerInit();
  powerDistributionInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= stateControllerTest();
  pass &= powerDistributionTest();

  return pass;
}

/* The stabilizer loop runs at 1kHz. It is the responsability or the different
 * functions to run slower by skipping call (ie. returning without modifying
 * the output structure).
 */
static void stabilizerTask(void* param)
{
  uint32_t tick = 0;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

    sensorsAcquire(&sensorData, tick);

    stateEstimator(&state, &sensorData, tick);

    //YHJ begin

    if (pathFollow_mode){                                                    
      commanderPFGetsetpoint(&setpoint, &state) ; 
    }
    else{
      commanderGetSetpoint(&setpoint, &state);  
    }
    
    //YHJ end

    //Original begin
    /*
    commanderGetSetpoint(&setpoint, &state);       
    */
    //Original end

    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

    stateController(&control, &sensorData, &state, &setpoint, tick);

    powerDistribution(&control); //YHJ for testing

    tick++;
  }
}

LOG_GROUP_START(setpointmode)
LOG_ADD(LOG_UINT8, mode_roll, &setpoint.mode.roll)
LOG_ADD(LOG_UINT8, mode_pitch, &setpoint.mode.pitch)
LOG_ADD(LOG_UINT8, mode_yaw, &setpoint.mode.yaw)
LOG_ADD(LOG_UINT8, mode_x, &setpoint.mode.x)
LOG_ADD(LOG_UINT8, mode_y, &setpoint.mode.y)
LOG_ADD(LOG_UINT8, mode_z, &setpoint.mode.z)
LOG_GROUP_STOP(setpointmode)

LOG_GROUP_START(setpoint_tgt_value)
LOG_ADD(LOG_FLOAT, tgt_x, &setpoint.position.x)
LOG_ADD(LOG_FLOAT, tgt_y, &setpoint.position.y)
LOG_ADD(LOG_FLOAT, tgt_z, &setpoint.position.z)
LOG_ADD(LOG_FLOAT, tgt_vx, &setpoint.velocity.x)
LOG_ADD(LOG_FLOAT, tgt_vy, &setpoint.velocity.y)
LOG_ADD(LOG_FLOAT, tgt_vz, &setpoint.velocity.z)
LOG_GROUP_STOP(setpoint_tgt_value)
/*
LOG_GROUP_START(setpoint_tgt_value)
LOG_ADD(LOG_FLOAT, tgt_roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, tgt_pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, tgt_yaw, &setpoint.attitude.yaw)
LOG_ADD(LOG_FLOAT, tgt_roll_rate, &setpoint.attitudeRate.roll)
LOG_ADD(LOG_FLOAT, tgy_pitch_rate, &setpoint.attitudeRate.pitch)
LOG_ADD(LOG_FLOAT, tgt_yaw_rate, &setpoint.attitudeRate.yaw)
LOG_ADD(LOG_FLOAT, tgt_x, &setpoint.position.x)
LOG_ADD(LOG_FLOAT, tgt_y, &setpoint.position.y)
LOG_ADD(LOG_FLOAT, tgt_z, &setpoint.position.z)
LOG_ADD(LOG_FLOAT, tgt_vx, &setpoint.velocity.x)
LOG_ADD(LOG_FLOAT, tgt_vy, &setpoint.velocity.y)
LOG_ADD(LOG_FLOAT, tgt_vz, &setpoint.velocity.z)
LOG_GROUP_STOP(setpoint_tgt_value)
*/

LOG_GROUP_START(crtlmeasure)
LOG_ADD(LOG_FLOAT, pos_x, &state.position.x)
LOG_ADD(LOG_FLOAT, pos_y, &state.position.y)
LOG_ADD(LOG_FLOAT, pos_z, &state.position.z)
LOG_GROUP_STOP(crtlmeasure)


LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_GROUP_STOP(stabilizer)

/*
LOG_GROUP_START(acc_XYZ)
LOG_ADD(LOG_FLOAT, x, &state.acc.x)
LOG_ADD(LOG_FLOAT, y, &state.acc.y)
LOG_ADD(LOG_FLOAT, z, &state.acc.z)
LOG_GROUP_STOP(acc_XYZ)
*/

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)


LOG_GROUP_START(motor_cmd)
LOG_ADD(LOG_INT16, roll, &control.roll)
LOG_ADD(LOG_INT16, pitch, &control.pitch)
LOG_ADD(LOG_INT16, yaw, &control.yaw)
LOG_ADD(LOG_FLOAT, thrust, &control.thrust)
LOG_GROUP_STOP(motor_cmd)


// Params for flight modes
PARAM_GROUP_START(pathfollowing)
PARAM_ADD(PARAM_UINT8, PF_mode, &pathFollow_mode)
PARAM_GROUP_STOP(pathfollowing)