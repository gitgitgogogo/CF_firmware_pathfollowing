/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
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
 * /// THIS IS TO RECEIVE KALMANFILTERED VICON DATA FROM CRAZIEFLIE ROS ///
 *
 */


#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#define VICON_POSITION_WDT_TIMEOUT_STABILIZE  M2T(1000)
#define VICON_POSITION_WDT_TIMEOUT_SHUTDOWN   M2T(5000)

/**
 * CRTP position data struct
 */

struct ViconPositionKFCrtpValues
{
  float viconX;
  float viconY;
  float viconZ;
  float viconVx;
  float viconVy;
  float viconVz;
} __attribute__((packed));


void viconPositionKFInit(void);
bool viconPositionKFTest(void);
uint32_t viconPositionKFGetInactivityTime(void);
//void viconPositionKFExtrxSet(const struct ViconPositionKFCrtpValues* val); //YHJ I don't think that we need this.
void viconPositionKFGetXYZ(float* viconX, float* viconY, float* viconZ, float* viconVx, float* viconVy, float* viconVz);

