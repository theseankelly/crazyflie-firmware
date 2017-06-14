
#include "stabilizer.h"
#include "estimator_complementary.h"
#include "sensfusion6.h"
#include "position_estimator.h"
#include "sensors.h"
#include "usec_time.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE

static uint64_t lastUpdateTimeUs = 0;
uint32_t estimatorLoopTimeUs = 0;
uint32_t sensorToEstLatencyUs = 0;

void estimatorComplementaryInit(void)
{
  sensfusion6Init();
}

bool estimatorComplementaryTest(void)
{
  bool pass = true;

  pass &= sensfusion6Test();

  return pass;
}

void estimatorComplementary(state_t *state, sensorData_t *sensorData, control_t *control, const uint32_t tick)
{
  sensorsAcquire(sensorData, tick); // Read sensors at full rate (1000Hz)
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {

    uint64_t timestamp = usecTimestamp();
    estimatorLoopTimeUs = (uint32_t)(timestamp - lastUpdateTimeUs);
    sensorToEstLatencyUs = (uint32_t)(timestamp - sensorData->gyro.timestamp);
    lastUpdateTimeUs = timestamp; 
    
    sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
                       sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
                       ATTITUDE_UPDATE_DT);
    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);

    state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.x,
                                                    sensorData->acc.y,
                                                    sensorData->acc.z);

    positionUpdateVelocity(state->acc.z, ATTITUDE_UPDATE_DT);
  }

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
