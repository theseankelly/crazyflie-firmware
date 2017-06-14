
#include "stabilizer.h"
#include "estimator_complementary.h"
#include "sensfusion6.h"
#include "position_estimator.h"
#include "sensors.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE

#ifdef PERFMONITOR
#include "usec_time.h"

static uint64_t lastUpdateTimeUs = 0;
uint32_t estimatorLoopTimeUs = 0;
uint32_t sensorToEstLatencyUs = 0;
#endif

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

// TODO: I can't decide where to stuff this variable -- static to this file or inside state_t??
static volatile uint64_t previousSensorTimestamp = 0;

void estimatorComplementary(state_t *state, sensorData_t *sensorData, control_t *control, const uint32_t tick)
{
  sensorsAcquire(sensorData, tick); // Read sensors at full rate (1000Hz)
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {

    // Compute DT in seconds using the gyro timestamp
    #if STAB_DEBUG_USE_REAL_DT 
    float dt = ((float)(sensorData->gyro.timestamp - previousSensorTimestamp) / 1000000.0f);
    #else
    float dt = ATTITUDE_UPDATE_DT;
    #endif
    previousSensorTimestamp = sensorData->gyro.timestamp;

#ifdef PERFMONITOR
    uint64_t timestamp = usecTimestamp();
    estimatorLoopTimeUs = (uint32_t)(timestamp - lastUpdateTimeUs);
    sensorToEstLatencyUs = (uint32_t)(timestamp - sensorData->gyro.timestamp);
    lastUpdateTimeUs = timestamp; 
#endif

    sensfusion6UpdateQ(sensorData->gyro.data.x, sensorData->gyro.data.y, sensorData->gyro.data.z,
                       sensorData->acc.data.x, sensorData->acc.data.y, sensorData->acc.data.z,
                       dt);

    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);

    state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.data.x,
                                                    sensorData->acc.data.y,
                                                    sensorData->acc.data.z);

    positionUpdateVelocity(state->acc.z, dt);
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
