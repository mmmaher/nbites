/**
 * @file TeamDataProvider.cpp
 * This file implements a module that provides the data received by team communication.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include <cstdint>
#include <limits>
#include "TeamDataProvider.h"
#include "Tools/Settings.h"
#include "Tools/Team.h"
#include "Tools/Math/Transformation.h"
#include <array>

MAKE_MODULE(TeamDataProvider, Cognition Infrastructure)

/**
 * This macro unpacks compressed representations. It reads
 * representationCompressed from the MessageQueue and unpacks it into
 * teammateData.array[robotNumber].
 */
#define UNPACK(representation, array) \
  representation##Compressed the##representation##Compressed; \
  message.bin >> the##representation##Compressed; \
  theTeammateDataBH.array[robotNumber] = the##representation##Compressed;

/**
 * This macro converts a timeStamp into local time via ntp.
 */
#define REMOTE_TO_LOCAL_TIME(timeStamp, robotNumber) \
  if(theTeammateDataBH.isBHumanPlayer[robotNumber]) \
  { \
    if(timeStamp) \
      timeStamp = ntp.getRemoteTimeInLocalTime(timeStamp); \
  }

PROCESS_WIDE_STORAGE(TeamDataProvider) TeamDataProvider::theInstance = 0;

TeamDataProvider::TeamDataProvider() :
  timeStamp(0), robotNumber(-1), lastSentTimeStamp(0)
{
  theInstance = this;
}

TeamDataProvider::~TeamDataProvider()
{
  theInstance = 0;
}

void TeamDataProvider::update(TeammateDataBH& teammateData)
{
  PLOT("module:TeamDataProvider:ntpOffset1", ntp.timeSyncBuffers[1].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset2", ntp.timeSyncBuffers[2].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset3", ntp.timeSyncBuffers[3].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset4", ntp.timeSyncBuffers[4].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset5", ntp.timeSyncBuffers[5].bestOffset);

  teammateData = theTeammateDataBH;

  fillOwnData(teammateData);

  teammateData.currentTimestamp = theFrameInfoBH.time;
  teammateData.numOfConnectedTeammates = 0;
  teammateData.numOfActiveTeammates = 0;
  teammateData.numOfFullyActiveTeammates = 0;
  teammateData.firstTeammate = TeammateDataBH::numOfPlayers;
  for(int i = TeammateDataBH::firstPlayer; i < TeammateDataBH::numOfPlayers; ++i)
  {
    if(i == theRobotInfoBH.number)
    {
      continue;
    }
    teammateData.isActive[i] = false;
    teammateData.isFullyActive[i] = false;
    // Check if network connection is working (-> connected):
    if(teammateData.timeStamps[i] && theFrameInfoBH.getTimeSince(teammateData.timeStamps[i]) < static_cast<int>(teammateData.networkTimeout))
    {
      teammateData.numOfConnectedTeammates++;
      // Check if teammate is not penalized (-> active):
      if(!teammateData.isPenalized[i] && theOwnTeamInfoBH.players[i - 1].penalty == PENALTY_NONE)
      {
        teammateData.numOfActiveTeammates++;
        teammateData.isActive[i] = true;
        if(i < static_cast<int>(teammateData.firstTeammate))
          teammateData.firstTeammate = i;
        // Check if teammate has not been fallen down or lost ground contact (-> fully active):
        if(teammateData.hasGroundContact[i] && teammateData.isUpright[i])
        {
          teammateData.numOfFullyActiveTeammates++;
          teammateData.isFullyActive[i] = true;
        }
      }
    }
  }
  if(teammateData.numOfConnectedTeammates)
    teammateData.wasConnected = theTeammateDataBH.wasConnected = true;
  teammateData.sendThisFrame =
#ifdef TARGET_ROBOT
    !(theMotionRequestBH.motion == MotionRequestBH::specialAction && theMotionRequestBH.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
    !(theMotionInfoBH.motion == MotionRequestBH::specialAction && theMotionInfoBH.specialActionRequest.specialAction == SpecialActionRequest::playDead)
#endif

    theFrameInfoBH.getTimeSince(lastSentTimeStamp) >= sendInterval;
  if(teammateData.sendThisFrame)
  {
    // Check if NTP has to respond
    ntp.doSynchronization(theFrameInfoBH.time, Global::getTeamOut());
    if(theFrameInfoBH.getTimeSince(lastSentTimeStamp) >= 2 * sendInterval)
      lastSentTimeStamp = theFrameInfoBH.time;
    else
      lastSentTimeStamp += sendInterval;
  }

  // calculate time to reach ball for teammates
  if(Global::getSettings().isDropInGame)
  {
    for(int i = TeammateDataBH::firstPlayer; i < TeammateDataBH::numOfPlayers; ++i)
    {
      if(i == theRobotInfoBH.number)
      {
        if(teammateData.behaviorStatus[i].role == RoleBH::striker)
          teammateData.behaviorStatus[i].estimatedTimeToReachBall = 0.f;
        else
          teammateData.behaviorStatus[i].estimatedTimeToReachBall = 1000.f;
      }
      else
      {
        // maybe do that only if the mesage was recceived this frame.
        teammateData.behaviorStatus[i].estimatedTimeToReachBall = estimatedDropInTimeToReachBall(teammateData, i);
      }
    }
    setRolesForTeammates(teammateData);
  }

  theTeammateDataBH = teammateData;
}

void TeamDataProvider::fillOwnData(TeammateData& teammateData)
{
  int ownNumber = theRobotInfoBH.number;
  teammateData.timeStamps[ownNumber] = theFrameInfoBH.time;
  teammateData.isActive[ownNumber] = true;
  teammateData.isFullyActive[ownNumber] = theGroundContactStateBH.contact && theFallDownStateBH.state == theFallDownStateBH.upright;
  teammateData.ballModels[ownNumber] = theBallModelBH;
  teammateData.robotPoses[ownNumber] = theRobotPoseBH;
  teammateData.robotsSideConfidence[ownNumber] = theSideConfidenceBH;
  //teammateData.behaviorStatus[ownNumber] = theBeBH;
  teammateData.isBHumanPlayer[ownNumber] = true;
  teammateData.isPenalized[ownNumber] = theRobotInfoBH.penalty != PENALTY_NONE;
  teammateData.hasGroundContact[ownNumber] = theGroundContactStateBH.contact;
  teammateData.isUpright[ownNumber] = theFallDownStateBH.state == theFallDownStateBH.upright;
  teammateData.timeLastGroundContact[ownNumber] = theFrameInfoBHBH.time;
  teammateData.cameraHeights[ownNumber] = theCameraMatrixBH.translation.z;
  teammateData.walkingTo[ownNumber] = theRobotPoseBH.translation;
  //teammateData.shootingTo[ownNumber] = ;

}

void TeamDataProvider::handleMessages(MessageQueue& teamReceiver)
{
  if(theInstance)
    teamReceiver.handleAllMessages(*theInstance);

  teamReceiver.clear();
}

bool TeamDataProvider::handleMessage(InMessage& message)
{
  // The robotNumber and the three flags hasGroundContact, isUpright and isPenalized should always be updated.
  switch(message.getMessageID())
  {
    case idNTPHeader:
      VERIFY(ntp.handleMessage(message));
      timeStamp = ntp.receiveTimeStamp;
      return false;
    case idNTPIdentifier:
    case idNTPRequest:
    case idNTPResponse:
      return ntp.handleMessage(message);

    case idRobot:
      message.bin >> robotNumber;
      if(robotNumber != theRobotInfoBH.number)
        if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
        {
          theTeammateDataBH.timeStamps[robotNumber] = timeStamp;
          if(!theTeammateDataBH.isBHumanPlayer[robotNumber])
          {
            char team;
            message.bin >> team;
            theTeammateDataBH.behaviorStatus[robotNumber].teamColor = (team == 0) ? BehaviorStatusBH::blue : BehaviorStatusBH::red;
          }
        }
      return true;

    case idDropInPlayer:
      if(robotNumber != theRobotInfoBH.number)
        if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
        {
          theTeammateDataBH.isBHumanPlayer[robotNumber] = false;
          theTeammateDataBH.timeStamps[robotNumber] = timeStamp = SystemCall::getCurrentSystemTime();
          char robotFallen;
          message.bin >> robotFallen; // 1 means that the robot is fallen, 0 means that the robot can play
          unsigned pseudoNetworkTimestanp;
          message.bin >> pseudoNetworkTimestanp;
          bool fallen = robotFallen >= 1;
          message.bin >> ballAge;
          theTeammateDataBH.isPenalized[robotNumber] = !(theOwnTeamInfoBH.players[robotNumber].penalty == PENALTY_NONE);
          if(theTeammateDataBH.hasGroundContact[robotNumber] && fallen)
            theTeammateDataBH.timeLastGroundContact[robotNumber] = pseudoNetworkTimestanp;
          theTeammateDataBH.hasGroundContact[robotNumber] = !fallen;
          theTeammateDataBH.cameraHeights[robotNumber] = theCameraMatrixBH.translation.z;
          theTeammateDataBH.obstacleClusters[robotNumber].obstacles.clear();
          theTeammateDataBH.obstacleModels[robotNumber].obstacles.clear();
        }
      return true;

    case idTeammateIntention:
      if(robotNumber != theRobotInfoBH.number)
        if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
          message.bin >> theTeammateDataBH.intention[robotNumber];
      return true;

    case idTeammateIsPenalized:
      if(robotNumber != theRobotInfoBH.number)
        if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
          message.bin >> theTeammateDataBH.isPenalized[robotNumber];
      return true;

    case idTeammateHasGroundContact:
      if(robotNumber != theRobotInfoBH.number)
        if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
        {
          message.bin >> theTeammateData.hasGroundContact[robotNumber];
          // This is a potentially evil quick workaround that should be replaced by a better handling of ground contacts of team mates
          // at many different places in our code! For a detailed problem description, ask Tim.
          if(!theTeammateData.hasGroundContact[robotNumber])
            theTeammateData.hasGroundContact[robotNumber] = theFrameInfoBH.getTimeSince(theTeammateDataBH.timeLastGroundContact[robotNumber]) < 2000;
        }
      return true;

    case idTeammateIsUpright:
      if(robotNumber != theRobotInfoBH.number)
        if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
          message.bin >> theTeammateDataBH.isUpright[robotNumber];
      return true;

    case idTeammateTimeSinceLastGroundContact:
      if(robotNumber != theRobotInfoBH.number)
        if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
        {
          message.bin >> theTeammateDataBH.timeLastGroundContact[robotNumber];
          REMOTE_TO_LOCAL_TIME(theTeammateDataBH.timeLastGroundContact[robotNumber]);
        }
      return true;
  }

  // The messages in the following switch block should only be updated
  // if hasGroundContact == true and isPenalized == false, because the information of this message
  // can only be reliable if the robot is actively playing.
  if(!theTeammateData.isPenalized[robotNumber] && theTeammateData.hasGroundContact[robotNumber])
  {
    switch(message.getMessageID())
    {
      case idTeammateBallModel:
        if(robotNumber != theRobotInfoBH.number)
          if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
          {
            BallModelBH oldBallModel = theTeammateDataBH.ballModels[robotNumber];
            UNPACK(BallModelBH, ballModels);
            if(!theTeammateDataBH.isBHumanPlayer[robotNumber] && (timeStamp - theTeammateDataBH.ballModels[robotNumber].timeWhenLastSeen) <= 16)
            {
              theTeammateDataBH.ballModels[robotNumber].lastPerception = oldBallModel.lastPerception;
            }
            REMOTE_TO_LOCAL_TIME(theTeammateDataBH.ballModels[robotNumber].timeWhenLastSeen, robotNumber);
            REMOTE_TO_LOCAL_TIME(theTeammateDataBH.ballModels[robotNumber].timeWhenDisappeared, robotNumber);

            BallModelBH& ballModel = theTeammateDataBH.ballModels[robotNumber];
            if(!theTeammateDataBH.isBHumanPlayer[robotNumber] &&
               (ballAge == -1 || ballModel.estimate.position.abs() < 30.f))
            {
              theTeammateDataBH.ballModels[robotNumber].timeWhenLastSeen = 0;
              theTeammateDataBH.ballModels[robotNumber].timeWhenDisappeared = 0;
            }
          }
        return true;

      case idTeammateObstacleModel:
        if(robotNumber != theRobotInfoBH.number)
          if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
          {
            UNPACK(ObstacleModelBH, obstacleModels);
          }
        return true;

      case idObstacleClusters:
        if(robotNumber != theRobotInfoBH.number)
        {
          if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
          {
            UNPACK(ObstacleClustersBH, obstacleClusters);
          }
        }
        return true;

      case idTeammateRobotPose:
        if(robotNumber != theRobotInfoBH.number)
          if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
          {
            UNPACK(RobotPoseBH, robotPoses);
          }
        return true;

      case idTeammateSideConfidence:
        if(robotNumber != theRobotInfoBH.number)
          if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
            message.bin >> theTeammateDataBH.robotsSideConfidence[robotNumber];
        return true;

      case idTeammateBehaviorStatus:
        if(robotNumber != theRobotInfoBH.number)
          if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
            message.bin >> theTeammateDataBH.behaviorStatus[robotNumber];
        return true;

      case idTeamCameraHeight:
        if(robotNumber != theRobotInfoBH.number)
          if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
            message.bin >> theTeammateDataBH.cameraHeights[robotNumber];
        return true;

      case idTeammateFieldCoverage:
        if(robotNumber != theRobotInfoBH.number)
          if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
          {
            FieldCoverageBH::GridIntervalBH& gridInterval = theTeammateDataBH.fieldCoverages[robotNumber];
            message.bin >> gridInterval;
            REMOTE_TO_LOCAL_TIME(gridInterval.timestamp, robotNumber);
          }
        return true;

      case idWalkTarget:
        if(robotNumber != theRobotInfoBH.number)
          if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
          {
            message.bin >> theTeammateDataBH.walkingTo[robotNumber];
          }
        return true;

      case idKickTarget:
        if(robotNumber != theRobotInfoBH.number)
          if(robotNumber >= TeammateDataBH::firstPlayer && robotNumber < TeammateDataBH::numOfPlayers)
          {
            message.bin >> theTeammateDataBH.shootingTo[robotNumber];
          }
        return true;
    }
  }
  return true;
}

void TeamDataProvider::setRolesForTeammates(TeammateDataBH& teammateData)
{
  //here the roles of the players are handled
  teammateData.behaviorStatus[theRobotInfoBH.number].role = RoleBH::striker;

  // all not handlned players get role none.
  for(int i = TeammateDataBH::firstPlayer; i < TeammateDataBH::numOfPlayers; ++i)
  {
    if(i != theRobotInfoBH.number)
    {
      teammateData.behaviorStatus[i].role = RoleBH::none;
    }
  }
}

float TeamDataProvider::estimatedDropInTimeToReachBall(TeammateDataBH& teammateData, int robotNumber)
{
  float timeToReachBall;
  // penalized, fallen robots with unsure or bad TeammateReliability have the max value to get to the ball
  if(!teammateData.isFullyActive[robotNumber] ||
     !(theTeammateReliabilityBH.states[robotNumber] == TeammateReliabilityBH::OK || theTeammateReliabilityBH.states[robotNumber] == TeammateReliabilityBH::GOOD))
  {
    return std::numeric_limits<float>::max();
  }

  // TODO maybe use endposition of Ball
  Vector2<> bPos = Transformation::robotToField(teammateData.robotPoses[robotNumber], teammateData.ballModels[robotNumber].estimate.position);
  // distance the robot has to walk to get to the ball. When it's currently walking to a target the way gos over the target.
  const float distanceToTarget = (teammateData.robotPoses[robotNumber].translation - teammateData.walkingTo[robotNumber]).abs();
  const float distance = distanceToTarget + (teammateData.walkingTo[robotNumber] - bPos).abs();
  timeToReachBall = distance / dropInSpeedOfRobot;

  // add time for the rotation to the ball
  timeToReachBall += std::abs(teammateData.ballModels[robotNumber].estimate.position.angle()) / fromDegrees(dropInRotationSpeedOfRobot);

  // when the robot currently dos not see the ball extra time is added
  timeToReachBall += theFrameInfoBH.getTimeSince(teammateData.ballModels[robotNumber].timeWhenLastSeen);

  // the currend striker gets a time bonus
  if(teammateData.behaviorStatus[robotNumber].role == RoleBH::striker)
  {
    timeToReachBall -= dropInBonusForTheStriker;
  }

  // add time when the relibility state is just OK
  if(theTeammateReliabilityBH.states[robotNumber] == TeammateReliabilityBH::OK)
  {
    timeToReachBall += dropInRelibilityOK;
  }
  else if(theTeammateReliabilityBH.states[robotNumber] == TeammateReliabilityBH::GOOD)
  {
    timeToReachBall += dropInRelibilityGOOD;
  }
  return timeToReachBall;
}
