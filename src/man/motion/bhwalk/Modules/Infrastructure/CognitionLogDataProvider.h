/**
* @file CognitionLogDataProvider.h
* This file declares a module that provides data replayed from a log file.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once
//TODO clean up includes
#include "Tools/ModuleBH/Module.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "LogDataProvider.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/ReceivedSPLStandardMessages.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeammateReliability.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Modeling/ObstacleWheel.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Modeling/LocalizationTeamBall.h"
#include "Representations/Infrastructure/LowFrameRateImage.h"
#include "Representations/Perception/RobotPercept.h"
#include "Representations/Perception/LineSpots.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Sensing/GroundContactState.h"

// MODULE(CognitionLogDataProvider)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT(CameraInfoBH)
//   USES(CameraInfoBH)
//   PROVIDES_WITH_OUTPUT(ImageBH)
//   REQUIRES(FieldDimensionsBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfoBH)
//   USES(FrameInfoBH)
//   REQUIRES(OwnTeamInfoBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT(LinePerceptBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallPerceptBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GoalPerceptBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallModelBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT(FilteredJointDataBH)
//   PROVIDES_WITH_DRAW(RobotsModelBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GroundTruthRobotPoseBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GroundTruthBallModelBH)
//   PROVIDES_WITH_DRAW(GroundTruthRobotsModelBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(CameraMatrixBH)
//   REQUIRES(ImageBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(ImageCoordinateSystemBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT(RobotPoseBH)
//   PROVIDES_WITH_MODIFY_AND_DRAW(SideConfidenceBH)
//   PROVIDES_WITH_DRAW(ObstacleModelBH)
//   PROVIDES(FilteredSensorDataBH)
//   PROVIDES_WITH_MODIFY(BehaviorControlOutputBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(MotionRequestBH)
//   PROVIDES_WITH_MODIFY(HeadMotionRequestBH)
//   PROVIDES_WITH_MODIFY(BehaviorLEDRequestBH)
//   PROVIDES_WITH_MODIFY(ArmMotionRequestBH)
//   PROVIDES_WITH_DRAW(CombinedWorldModelBH)
//   PROVIDES_WITH_MODIFY(MotionInfoBH)
//   PROVIDES_WITH_MODIFY_AND_DRAW(RobotHealthBH)
//   PROVIDES_WITH_MODIFY_AND_DRAW(FieldBoundaryBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT(ActivationGraphBH)
//   PROVIDES_WITH_DRAW(ObstacleWheelBH)
//   PROVIDES(ColorReferenceBH)
//   PROVIDES_WITH_DRAW(ObstacleSpotsBH)
//   PROVIDES_WITH_OUTPUT_AND_DRAW(ThumbnailBH)
//   PROVIDES_WITH_DRAW(BodyContourBH)
// END_MODULE

MODULE(CognitionLogDataProvider,
{,
  REQUIRES(OwnTeamInfo),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(OwnTeamInfo),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(OpponentTeamInfo),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GameInfo),
  PROVIDES_WITH_MODIFY(RobotInfo),
  PROVIDES_WITH_OUTPUT(AudioData),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(CameraInfo),
  USES(CameraInfo),
  PROVIDES(CameraInfoFullRes),
  PROVIDES_WITH_OUTPUT(Image),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfo),
  USES(FrameInfo),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(LinePercept),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallPercept),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GoalPercept),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallModel),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FilteredJointData),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(CameraMatrix),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(ImageCoordinateSystem),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(RobotPose),
  PROVIDES_WITH_MODIFY_AND_DRAW(SideConfidence),
  PROVIDES_WITH_DRAW(ObstacleModel),
  PROVIDES(FilteredSensorData),
  PROVIDES_WITH_MODIFY(BehaviorControlOutput),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(MotionRequest),
  PROVIDES_WITH_MODIFY(HeadMotionRequest),
  PROVIDES_WITH_MODIFY(BehaviorLEDRequest),
  PROVIDES_WITH_MODIFY(ArmMotionRequest),
  PROVIDES_WITH_DRAW(CombinedWorldModel),
  PROVIDES_WITH_MODIFY(MotionInfo),
  PROVIDES_WITH_MODIFY_AND_DRAW(RobotHealth),
  PROVIDES_WITH_MODIFY_AND_DRAW(FieldBoundary),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ActivationGraph),
  PROVIDES_WITH_DRAW(ObstacleWheel),
  PROVIDES_WITH_DRAW(BodyContour),
  PROVIDES_WITH_MODIFY_AND_DRAW(RobotPercept),
  PROVIDES_WITH_MODIFY(GroundTruthWorldState),
  PROVIDES_WITH_DRAW(LineSpots),
  PROVIDES_WITH_MODIFY(Odometer),
  PROVIDES_WITH_MODIFY(GroundContactState),
  PROVIDES_WITH_MODIFY_AND_DRAW(LocalizationTeamBall),
  PROVIDES(TeammateDataCompressed),
  PROVIDES_WITH_DRAW(TeammateData),
  PROVIDES_WITH_DRAW(TeammateReliability),
  PROVIDES_WITH_MODIFY(ReceivedSPLStandardMessages),
});

class CognitionLogDataProvider : public CognitionLogDataProviderBase, public LogDataProvider
{
private:
  static PROCESS_WIDE_STORAGE(CognitionLogDataProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
  bool frameDataComplete; /**< Were all messages of the current frame received? */
  BehaviorControlOutputBH behaviorControlOutput;
  ImageBH lastImage[CameraInfoBH::numOfCameras];

  DECLARE_DEBUG_IMAGE(corrected);

#define DISTANCE 300

  void update(ImageBH& image)
   {
    if(representationBuffer[idLowFrameRateImage])
    {
      CameraInfoBH& info = (CameraInfoBH&) *representationBuffer[idCameraInfo];
      LowFrameRateImageBH& lfri = (LowFrameRateImageBH&) *representationBuffer[idLowFrameRateImage];
      if(lfri.imageUpdated)
      {
        lastImage[info.camera] = lfri.image;
        image = lfri.image;
      }
      else
        image = lastImage[info.camera];
    }
    else if(representationBuffer[idImage])
      image = *((ImageBH*)representationBuffer[idImage]);

    DECLARE_DEBUG_DRAWING3D("representation:ImageBH", "camera");
    IMAGE3D("representation:ImageBH", DISTANCE, 0, 0, 0, 0, 0,
            DISTANCE * theCameraInfoBH.width / theCameraInfoBH.focalLength,
            DISTANCE * theCameraInfo.height / theCameraInfoBH.focalLength,
            image);
    DEBUG_RESPONSE("representation:JPEGImage", OUTPUT(idJPEGImage, bin, JPEGImage(image)););
  }
  UPDATE(RobotPerceptBH)
  UPDATE(OwnTeamInfoBH)
  UPDATE(OpponentTeamInfoBH)
  UPDATE(GameInfoBH)
  UPDATE(RobotInfoBH)
  UPDATE(AudioDataBH)
  UPDATE(CameraInfoBH)
  void update(CameraInfoFullResBH& cameraInfoFullRes)
  {
    if(representationBuffer[idCameraInfo])
      cameraInfoFullRes = *((CameraInfoBH*) representationBuffer[idCameraInfo]);
  }
  UPDATE(FrameInfoBH)
  UPDATE(FieldBoundaryBH);
  UPDATE(ActivationGraphBH)
  UPDATE(ObstacleWheelBH)
  UPDATE(BodyContourBH);
  UPDATE(RobotHealthBH);

  UPDATE(BehaviorControlOutputBH);
  void update(MotionRequestBH& motionRequest) {motionRequest = behaviorControlOutput.motionRequest;}
  void update(HeadMotionRequestBH& headMotionRequest) {headMotionRequest = behaviorControlOutput.headMotionRequest;}
  void update(BehaviorLEDRequestBH& behaviorLEDRequest) {behaviorLEDRequest = behaviorControlOutput.behaviorLEDRequest;}
  void update(ArmMotionRequestBH& armMotionRequest) {armMotionRequest = behaviorControlOutput.armMotionRequest;}

  UPDATE(CombinedWorldModelBH)
  UPDATE(ObstacleModelBH)
  UPDATE(FilteredSensorDataBH)
  UPDATE(LinePerceptBH)
  UPDATE(LineSpotsBH)
  UPDATE(BallPerceptBH)
  UPDATE(GoalPerceptBH)
  UPDATE(BallModelBH)
  UPDATE(LocalizationTeamBallBH)
  UPDATE(FilteredJointDataBH)
  UPDATE(CameraMatrixBH)
  UPDATE(TeammateDataCompressedBH)
  UPDATE(TeammateDataBH)
  UPDATE(TeammateReliabilityBH)
  UPDATE2(ImageCoordinateSystemBH,
  {
    _ImageCoordinateSystem.setCameraInfo(theCameraInfoBH);
    DECLARE_DEBUG_DRAWING("loggedHorizon", "drawingOnImage"); // displays the horizon
    ARROW("loggedHorizon",
          _ImageCoordinateSystem.origin.x,
          _ImageCoordinateSystem.origin.y,
          _ImageCoordinateSystem.origin.x + _ImageCoordinateSystem.rotation.c[0].x * 50,
          _ImageCoordinateSystem.origin.y + _ImageCoordinateSystem.rotation.c[0].y * 50,
          0, Drawings::ps_solid, ColorRGBA(255, 0, 0));
    ARROW("loggedHorizon",
          _ImageCoordinateSystem.origin.x,
          _ImageCoordinateSystem.origin.y,
          _ImageCoordinateSystem.origin.x + _ImageCoordinateSystem.rotation.c[1].x * 50,
          _ImageCoordinateSystem.origin.y + _ImageCoordinateSystem.rotation.c[1].y * 50,
          0, Drawings::ps_solid, ColorRGBA(255, 0, 0));
    COMPLEX_DEBUG_IMAGE(corrected,
    {
      ImageBH* i = (ImageBH*) representationBuffer[idImage];
      if(i)
      {
        INIT_DEBUG_IMAGE_BLACK(corrected, theCameraInfoBH.width, theCameraInfo.height);
        int yDest = -_ImageCoordinateSystem.toCorrectedCenteredNeg(0, 0).y;
        for(int ySrc = 0; ySrc < theCameraInfo.height; ++ySrc)
          for(int yDest2 = -_ImageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).y; yDest <= yDest2; ++yDest)
          {
            int xDest = -_ImageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).x;
            for(int xSrc = 0; xSrc < theCameraInfoBH.width; ++xSrc)
            {
              for(int xDest2 = -_ImageCoordinateSystem.toCorrectedCenteredNeg(xSrc, ySrc).x; xDest <= xDest2; ++xDest)
              {
                DEBUG_IMAGE_SET_PIXEL_YUV(corrected, xDest + int(theCameraInfoBH.opticalCenter.x + 0.5f),
                yDest + int(theCameraInfoBH.opticalCenter.y + 0.5f),
                (*i)[ySrc][xSrc].y,
                (*i)[ySrc][xSrc].cb,
                (*i)[ySrc][xSrc].cr);
              }
            }
          }
        SEND_DEBUG_IMAGE(corrected);
      }
    });
  })
  UPDATE(RobotPoseBH)
  UPDATE(SideConfidenceBH)
  UPDATE(MotionInfoBH)
  UPDATE(GroundTruthWorldStateBH)
  UPDATE(OdometerBH)
  UPDATE(GroundContactStateBH)
  UPDATE(ReceivedSPLStandardMessagesBH)


  /**
  * The method is called for every incoming debug message by handleMessage.
  * @param message An interface to read the message from the queue.
  * @return Was the message handled?
  */
  bool handleMessage2(InMessage& message);

public:
  /**
  * Default constructor.
  */
  CognitionLogDataProvider();

  /**
  * Destructor.
  */
  ~CognitionLogDataProvider();

  /**
  * The method is called for every incoming debug message.
  * @param message An interface to read the message from the queue.
  * @return Was the message handled?
  */
  static bool handleMessage(InMessage& message);

  /**
  * The method returns whether idProcessFinished was received.
  * @return Were all messages of the current frame received?
  */
  static bool isFrameDataComplete();
};
