/**
* @file Modules/Infrastructure/RobotHealthProvider.h
* This file declares a module that provides information about the robot's health.
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Tools/ModuleBH/Module.h"
#include "Tools/RingBufferWithSum.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/GoalPercept.h"
#ifdef TARGET_ROBOT
#include "Platform/Linux/NaoBody.h"
#endif

MODULE(RobotHealthProvider,
{,
  REQUIRES(BallPerceptBH),
  REQUIRES(LinePerceptBH),
  REQUIRES(GoalPerceptBH),
  REQUIRES(MotionRobotHealthBH),
  REQUIRES(FilteredSensorDataBH),
  REQUIRES(FrameInfoBH),
  PROVIDES_WITH_MODIFY_AND_DRAW(RobotHealthBH),
  LOADS_PARAMETERS(
  {,
    (char) batteryLow, /**< The voltage below which the robot gives low battery warnings. */
    (int) temperatureHigh, /**< The temperature the robot starts complaining about the temperature. */
    (bool) enableName,
  }),
});

/**
* @class RobotHealthProvider
* A module that provides information about the robot's health
*/
class RobotHealthProvider : public RobotHealthProviderBase
{
public:
  /** Constructor. */
  RobotHealthProvider();

private:
  STREAMABLE(BuildInfoBH,
  {,
    (RobotHealthBH, Configuration)(Develop) configuration, /**< The configuration that was deployed. */
    (std::string)("unknown") hash, /**< The first 5 digits of the hash of the git HEAD that was deployed. */
    (bool)(false) clean, /**< Was the working copy clean when it was deployed? */
  });

  BuildInfoBH buildInfo; /**< Information about the revision that was deployed. */
  RingBufferWithSumBH<unsigned, 30> timeBuffer; /**< Buffered timestamps of previous executions */
  unsigned lastExecutionTime;
  unsigned lastRelaxedHealthComputation;
  unsigned startBatteryLow; /**< Last time the battery state was not low. */
  float lastBatteryLevel;
  bool batteryVoltageFalling;
  unsigned highTemperatureSince;
#ifdef TARGET_ROBOT
  NaoBody naoBody;
  unsigned int lastBodyTemperatureReadTime;
  unsigned int lastWlanCheckedTime;
#endif

  /** The main function, called every cycle
  * @param robotHealth The data struct to be filled
  */
  void update(RobotHealthBH& robotHealth);
};
