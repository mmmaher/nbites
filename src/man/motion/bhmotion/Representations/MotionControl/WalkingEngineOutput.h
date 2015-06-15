/**
* @file Representations/MotionControl/WalkingEngineOutput.h
* This file declares a class that represents the output of modules generating motion.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/WalkRequest.h"
#include "Tools/Math/Vector3.h"

/**
* @class WalkingEnigeOutput
* A class that represents the output of the walking engine.
*/
STREAMABLE_WITH_BASE(WalkingEngineOutputBH, JointRequestBH,
{,
  (bool)(true) standing, /**< Whether the robot is standing or walking */
  (Pose2DBH) speed, /**< The current walking speed in mm/s and rad/s. */
  (Pose2DBH) odometryOffset, /**< The body motion performed in this step. */
  (Pose2DBH) upcomingOdometryOffset, /**< The remaining odometry offset for the currently executed step. */
  (bool)(false) upcomingOdometryOffsetValid, /**< Whether the \c upcomingOdometryOffset is precise enough to be used */
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
  (float)(0) positionInWalkCycle, /**< The current position in the walk cycle in the range [0..1[. */
  (float)(0) instability, /**< An evaluation of the current walk stability. */
  (WalkRequest) executedWalk, /**< The walk currently executed. */
});

/**
 * @class WalkingEngineOutputBH
 * A class that represents the output of the walking engine.
 */
class WalkingEngineStandOutputBH : public JointRequestBH {};
