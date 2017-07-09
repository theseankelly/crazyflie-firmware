
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "sensfusion6.h"
#include "position_estimator.h"

#include "usec_time.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE

static uint64_t lastUpdateUs = 0;

void stateEstimatorInit(void)
{
  lastUpdateUs = usecTimestamp();
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
  // Compute dt
#if 1 // Use sensor data timestamp for basis of calculation
  float dt = ((float)(sensorData->gyro.timestamp - lastUpdateUs))/1000000.0f;
  lastUpdateUs = sensorData->gyro.timestamp;
#else // use t_now for basis of caluclation
  uint64_t timestamp = usecTimestamp();
  float dt = ((float)(timestamp - lastUpdateUs))/1000000.0f;
  lastUpdateUs = timestamp;
#endif

  sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
                     sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
                     dt);
  sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);

  state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.x,
                                                  sensorData->acc.y,
                                                  sensorData->acc.z);

  positionUpdateVelocity(state->acc.z, ATTITUDE_UPDATE_DT);

  if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick)) {
    // If position sensor data is preset, pass it throught
    // FIXME: The position sensor shall be used as an input of the estimator
    if (sensorData->position.timestamp) {
      state->position = sensorData->position;
    } else {
      positionEstimate(state, sensorData, POS_UPDATE_DT, tick);
    }
  }
}
