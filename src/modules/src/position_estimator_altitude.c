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
 * position_estimator_altitude.c: Altitude-only position estimator
 */

#include "log.h"
#include "param.h"
#include "num.h"
#include "position_estimator.h"
#include "vicon_position.h"  // YHJ
#include "vicon_positionKF.h"  // YHJ 

#define G 9.81;

static float viconX = 0.0f;  // YHJ begin
static float viconY = 0.0f;  //
static float viconZ = 0.0f;  // 
static float viconYaw = 0.0f;  // YHJ end

static float viconVx = 0.0f;  // YHJ begin
static float viconVy = 0.0f;  //
static float viconVz = 0.0f;  // YHJ end


struct selfState_s {
  //YHJ begin
  float estimatedX; 
  float velocityX;  
  float estimatedY; 
  float velocityY;
  //YHJ end  
  float estimatedZ; // The current Z estimate, has same offset as asl
  float velocityZ; // Vertical speed (world frame) integrated from vertical acceleration (m/s)
  float estAlpha;
  float estAlpha_velocity;
  float velocityFactor;
  float vAccDeadband; // Vertical acceleration deadband
  float velZAlpha;   // Blending factor to avoid vertical speed to accumulate error
};

static struct selfState_s state = {
  //YHJ begin
  .estimatedX = 0.0,
  .velocityX = 0.0,
  .estimatedY = 0.0,
  .velocityY = 0.0,
  //YHJ end
  .estimatedZ = 0.0,
  .velocityZ = 0.0,
  .estAlpha = 0.20,
  .estAlpha_velocity = 0.80,
  .velocityFactor = 1.0,
  .vAccDeadband = 0.04,
  .velZAlpha = 0.995
};

static void positionEstimateInternal(state_t* estimate, float asl, float dt, struct selfState_s* state);
static void positionUpdateVelocityInternal(float accWZ, float dt, struct selfState_s* state);

//YHJ begin
static void positionUpdateVelocity_X_Internal(float accWX, float dt, struct selfState_s* state);
static void positionUpdateVelocity_Y_Internal(float accWY, float dt, struct selfState_s* state);
//YHJ end


void positionEstimate(state_t* estimate, float asl, float dt) {
  positionEstimateInternal(estimate, asl, dt, &state);
}

void positionUpdateVelocity(float accWZ, float dt) {
  positionUpdateVelocityInternal(accWZ, dt, &state);
}

//YHJ begin
void positionUpdateVelocity_X(float accWX, float dt) {
  positionUpdateVelocity_X_Internal(accWX, dt, &state);
}

void positionUpdateVelocity_Y(float accWY, float dt) {
  positionUpdateVelocity_Y_Internal(accWY, dt, &state);
}
//YHJ end


static void positionEstimateInternal(state_t* estimate, float asl, float dt, struct selfState_s* state) {
  // YHJ begin 
  // Position Estimation using Vicon and IMU
  //viconPositionGetXYZ(&viconX, &viconY, &viconZ, &viconYaw);   // YHJ 
  viconPositionKFGetXYZ(&viconX, &viconY, &viconZ, &viconVx, &viconVy, &viconVz);   // YHJ with Kalman Filtering
  // The below is a simple linear complementary filter
  state->estimatedX = state->estAlpha * state->estimatedX +
                     (1.0 - state->estAlpha) * viconX +    
                     state->velocityFactor * state->velocityX * dt;
  // The below is a simple linear complementary filter
  state->estimatedY = state->estAlpha * state->estimatedY +
                     (1.0 - state->estAlpha) * viconY +    
                     state->velocityFactor * state->velocityY * dt;

  //YHJ end
  
  /* The below is the original code which combined asl(above sea line) and IMU.

  state->estimatedZ = state->estAlpha * state->estimatedZ +
                     (1.0 - state->estAlpha) * asl +   //YHJ here asl(above sea level) is vertical position[m] from  barometer.
                     state->velocityFactor * state->velocityZ * dt;

  The original end */ 


  // YHJ begin
  state->estimatedZ = state->estAlpha * state->estimatedZ +
                     (1.0 - state->estAlpha) * viconZ +   //YHJ here asl(above sea level) is vertical position[m] from  barometer.
                     state->velocityFactor * state->velocityZ * dt;                 
  // YHJ end  



  estimate->position.x = state->estimatedX;  // in original this set to zero
  estimate->position.y = state->estimatedY;  // in original this set to zero
  estimate->position.z = state->estimatedZ;

}


static void positionUpdateVelocityInternal(float accWZ, float dt, struct selfState_s* state) {
  //Original
  /*
  state->velocityZ += deadband(accWZ, state->vAccDeadband) * dt * G;
  state->velocityZ *= state->velZAlpha;
  */

  state->velocityZ = state->estAlpha_velocity * state->velocityZ +
                     (1.0 - state->estAlpha_velocity) *viconVz +
                     state->velocityFactor * deadband(accWZ, state->vAccDeadband) * dt * G 
}


//YHJ begin
static void positionUpdateVelocity_X_Internal(float accWX, float dt, struct selfState_s* state) {
  //YHJ
  /*
  state->velocityX += deadband(accWX, state->vAccDeadband) * dt * G;
  state->velocityX *= state->velZAlpha;
  */
  // Because we have Vicon data, I changed it to a complementary filter
  state->velocityX = state->estAlpha_velocity * state->velocityX +
                     (1.0 - state->estAlpha_velocity) *viconVx +
                     state->velocityFactor * deadband(accWX, state->vAccDeadband) * dt * G

}

static void positionUpdateVelocity_Y_Internal(float accWY, float dt, struct selfState_s* state) {
  //YHJ
  /*
  state->velocityY += deadband(accWY, state->vAccDeadband) * dt * G;
  state->velocityY *= state->velZAlpha;
  */
  // Because we have Vicon data, I changed it to a complementary filter
  state->velocityY = state->estAlpha_velocity * state->velocityY +
                     (1.0 - state->estAlpha_velocity) *viconVy +
                     state->velocityFactor * deadband(accWY, state->vAccDeadband) * dt * G
}
//YHJ end



LOG_GROUP_START(posEstimator)
LOG_ADD(LOG_FLOAT, X, &state.estimatedX)  //YHJ add
LOG_ADD(LOG_FLOAT, Vx, &state.velocityX)    //YHJ add
LOG_ADD(LOG_FLOAT, Y, &state.estimatedY)  //YHJ add
LOG_ADD(LOG_FLOAT, Vy, &state.velocityY)    //YHJ add
LOG_ADD(LOG_FLOAT, Z, &state.estimatedZ)
LOG_ADD(LOG_FLOAT, Vz, &state.velocityZ)
LOG_GROUP_STOP(posEstimator)

LOG_GROUP_START(viconData)  //YHJ begin
LOG_ADD(LOG_FLOAT, X, &viconX)
LOG_ADD(LOG_FLOAT, Y, &viconY)
LOG_ADD(LOG_FLOAT, Z, &viconZ)
LOG_ADD(LOG_FLOAT, Vx, &viconVx)
LOG_ADD(LOG_FLOAT, Vy, &viconVy)
LOG_ADD(LOG_FLOAT, Vz, &viconVz)
LOG_GROUP_STOP(viconData)  //YHJ end


PARAM_GROUP_START(posEst)
PARAM_ADD(PARAM_FLOAT, estAlpha, &state.estAlpha)
PARAM_ADD(PARAM_FLOAT, velFactor, &state.velocityFactor)
PARAM_ADD(PARAM_FLOAT, velZAlpha, &state.velZAlpha)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &state.vAccDeadband)
PARAM_GROUP_STOP(posEst)
