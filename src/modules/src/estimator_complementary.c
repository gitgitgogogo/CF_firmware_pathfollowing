
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "sensfusion6.h"
#include "position_estimator.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE

void stateEstimatorInit(void)
{
  sensfusion6Init();
}

bool stateEstimatorTest(void)
{
  bool pass = true;

  pass &= sensfusion6Test();

  return pass;
}

void stateEstimator(state_t *state, const sensorData_t *sensorData, const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
    sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
                       sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
                       ATTITUDE_UPDATE_DT);
    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);

    state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.x,
                                                    sensorData->acc.y,//
                                                    sensorData->acc.z);
    //YHJ begin
    //They update accleration vector respect to the inertial frame, XYZ. 
    state->acc.x = sensfusion6GetAccX(sensorData->acc.x,
                                      sensorData->acc.y,
                                      sensorData->acc.z);

    state->acc.y = sensfusion6GetAccY(sensorData->acc.x,
                                      sensorData->acc.y,
                                      sensorData->acc.z);
    //YHJ end

    positionUpdateVelocity(state->acc.z, ATTITUDE_UPDATE_DT);

    //YHJ begin
    positionUpdateVelocity_X(state->acc.x, ATTITUDE_UPDATE_DT);
    positionUpdateVelocity_Y(state->acc.y, ATTITUDE_UPDATE_DT);
    //YHJ end

  }

  if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick)) {
    // If position sensor data is preset, pass it throught
    // FIXME: The position sensor shall be used as an input of the estimator
    if (sensorData->position.timestamp) {
      state->position = sensorData->position;
    } else {
      positionEstimate(state, sensorData->baro.asl, POS_UPDATE_DT);
    }
  }
}
