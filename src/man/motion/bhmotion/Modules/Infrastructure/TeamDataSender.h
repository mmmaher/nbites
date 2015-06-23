/**
* @file TeamDataSender.h
* Declaration of module TeamDataSender
* @author Colin Graf
*/

#pragma once

#include "Tools/ModuleBH/Module.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Modeling/ObstacleClusters.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

MODULE(TeamDataSender,
{,
  REQUIRES(BallModelBH),
  REQUIRES(BehaviorControlOutputBH),
  REQUIRES(CameraMatrixBH),
  REQUIRES(FallDownStateBH),
  REQUIRES(FrameInfoBH),
  REQUIRES(GoalPerceptBH),
  REQUIRES(GroundContactStateBH),
  REQUIRES(MotionInfoBH),
  REQUIRES(MotionRequestBH),
  REQUIRES(ObstacleClustersBH),
  REQUIRES(ObstacleModelBH),
  REQUIRES(OwnTeamInfoBH),
  REQUIRES(RobotHealthBH),
  REQUIRES(RobotInfoBH),
  REQUIRES(RobotPoseBH),
  REQUIRES(SideConfidenceBH),
  REQUIRES(TeammateDataBH),
  PROVIDES(TeamDataSenderOutputBH),
  LOADS_PARAMETERS(
  {,
    (unsigned) maxNumberOfRobotsToSend, /**< Do not send more robots than this. */
    (unsigned) maxNumberOfObstaclesToSend, /**< Do not send more obstacles than this. */
    (unsigned) maxNumberOfObstacleClustersToSend, /**< Do not send more obstacle clusters than this. */
  }),
});

/**
* @class TeamDataSender
* A modules for sending some representation to teammates
*/
class TeamDataSender : public TeamDataSenderBase
{
public:

  /** Default constructor */
  TeamDataSender() : TeamDataSenderBase("teamDataSender.cfg"), sendFrames(0) {}

private:
  unsigned int sendFrames; /** Quantity of frames in which team data was sent */

  /**
  * The update function called in each cognition process cycle
  * @param teamDataSenderOutput An empty output representation
  */
  virtual void update(TeamDataSenderOutputBH& teamDataSenderOutput);
};
