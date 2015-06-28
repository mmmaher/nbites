/**
* @file TeamDataSender.cpp
* Implementation of module TeamDataSender
* @author Colin Graf
*/

#include "Representations/Modeling/ObstacleClusters.h"
#include "TeamDataSender.h"
#include "Tools/Team.h"

MAKE_MODULE(TeamDataSender, Cognition Infrastructure)

void TeamDataSender::update(TeamDataSenderOutputBH& teamDataSenderOutput)
{
  if(theTeammateDataBH.sendThisFrame)
  {
    ++sendFrames;

    TEAM_OUTPUT(idRobot, bin, theRobotInfoBH.number);
    TEAM_OUTPUT(idTeam, bin, theOwnTeamInfoBH.teamColor);

    // Own pose information and ball observation:
    TEAM_OUTPUT(idTeammateRobotPose, bin, RobotPoseCompressed(theRobotPoseBH));
    TEAM_OUTPUT(idTeammateSideConfidence, bin, theSideConfidenceBH);
    TEAM_OUTPUT(idTeammateBallModel, bin, BallModelCompressed(theBallModelBH));
    TEAM_OUTPUT(idTeammateBallAge, bin, (theBallModelBH.timeWhenLastSeen ? theFrameInfoBH.getTimeSince(theBallModelBH.timeWhenLastSeen) : -1));
    TEAM_OUTPUT(idTeammateGoalPercept, bin, theGoalPerceptBH);

    // Obstacle stuff
    TEAM_OUTPUT(idObstacleClusters, bin, ObstacleClustersCompressed(theObstacleClustersBH, maxNumberOfObstacleClustersToSend));
    TEAM_OUTPUT(idTeammateObstacleModel, bin, ObstacleModelCompressed(theObstacleModelBH, maxNumberOfObstaclesToSend));

    // Robot status
    TEAM_OUTPUT(idTeammateIsPenalized, bin, (theRobotInfoBH.penalty != PENALTY_NONE));
    TEAM_OUTPUT(idTeammateHasGroundContact, bin, (theGroundContactStateBH.contact && theMotionInfoBH.motion != MotionInfoBH::getUp && theMotionRequestBH.motion != MotionRequestBH::getUp));
    TEAM_OUTPUT(idTeammateIsUpright, bin, (theFallDownStateBH.state == theFallDownStateBH.upright));


    if(theGroundContactStateBH.contact)
    {
      TEAM_OUTPUT(idTeammateTimeSinceLastGroundContact, bin, theFrameInfoBH.time);
    }
    TEAM_OUTPUT(idTeamCameraHeight, bin, theCameraMatrixBH.translation.z);

    if(sendFrames % 20 == 0)
      TEAM_OUTPUT(idRobotHealth, bin, theRobotHealthBH);

    // send intention
    if(theSideConfidenceBH.confidenceState == SideConfidenceBH::CONFUSED)
    {
      TEAM_OUTPUT(idTeammateIntention, bin, DROPIN_INTENTION_LOST);
    }
    else
    {
      TEAM_OUTPUT(idTeammateIntention, bin, TeammateData::getIntentionForRole(theBehaviorControlOutputBH.behaviorStatus.role));
    }
  }
}
