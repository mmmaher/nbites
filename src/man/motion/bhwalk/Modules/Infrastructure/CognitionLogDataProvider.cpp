/**
* @file CognitionLogDataProvider.cpp
* This file implements a module that provides data replayed from a log file.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "CognitionLogDataProvider.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Tools/Settings.h"
#include <vector>

PROCESS_WIDE_STORAGE(CognitionLogDataProvider) CognitionLogDataProvider::theInstance = 0;

#define ASSIGN(target, source) \
  ALLOC(target) \
  (target&) *representationBuffer[id##target] = (target&) *representationBuffer[id##source];

CognitionLogDataProvider::CognitionLogDataProvider() :
  LogDataProvider(),
  frameDataComplete(false)
{
  theInstance = this;
}

CognitionLogDataProvider::~CognitionLogDataProvider()
{
  theInstance = 0;
}

bool CognitionLogDataProvider::handleMessage(InMessage& message)
{
  return theInstance && theInstance->handleMessage2(message);
}

bool CognitionLogDataProvider::isFrameDataComplete()
{
  if(!theInstance)
    return true;
  else if(theInstance->frameDataComplete)
  {
    OUTPUT(idLogResponse, bin, '\0');
    theInstance->frameDataComplete = false;
    return true;
  }
  else
    return false;
}

bool CognitionLogDataProvider::handleMessage2(InMessage& message)
{
  switch(message.getMessageID())
  {
    HANDLE(OwnTeamInfoBH)
    HANDLE(OpponentTeamInfoBH)
    HANDLE(GameInfoBH)
    HANDLE2(RobotInfoBH, ((RobotInfoBH&) *representationBuffer[idRobotInfo]).number = Global::getSettings().playerNumber;)
    HANDLE(AudioDataBH)
    HANDLE2(ImageBH,
    {
      ALLOC(FrameInfoBH)
      FrameInfoBH& frameInfo = (FrameInfoBH&) *representationBuffer[idFrameInfo];
      const ImageBH& image = (const ImageBH&) *representationBuffer[idImage];
      frameInfo.cycleTime = (float) (image.timeStamp - frameInfo.time) * 0.001f;
      frameInfo.time = image.timeStamp;
    })
    HANDLE(LowFrameRateImageBH)
    HANDLE(CameraInfoBH)
    HANDLE2(FrameInfoBH,
    {
      ALLOC(ImagBH)
      ImageBH& image = (ImageBH&) *representationBuffer[idImage];
      const FrameInfoBH& frameInfo = (FrameInfoBH&) *representationBuffer[idFrameInfo];
      image.timeStamp = frameInfo.time;
    })
    HANDLE(LinePerceptBH)
    HANDLE(ActivationGraphBH)
    HANDLE(BallPerceptBH)
    HANDLE(GoalPerceptBH)
    HANDLE(FieldBoundaryBH)
    HANDLE(BallModelBH)
    HANDLE(ObstacleWheelBH)
    HANDLE(BodyContourBH)
    HANDLE2(ThumbnailBH,
    {
      ALLOC(ImageBH)
      ThumbnailBH& thumbnail = (ThumbnailBH&) *representationBuffer[idThumbnail];
      thumbnail.toImage((ImageBH&) *representationBuffer[idImage]);
      return true;
    })
    HANDLE(TeammateReliabilityBH)
    HANDLE2(TeammateDataCompressedBH,
    {
      ALLOC(TeammateDataBH)
      TeammateDataBH& teammateData = (TeammateDataBH&) *representationBuffer[idTeammateData];
      TeammateDataCompressedBH& teammateDataCompressed = (TeammateDataCompressedBH&) *representationBuffer[idTeammateDataCompressed];
      teammateData = TeammateData(teammateDataCompressed);
    })
    HANDLE(RobotHealth)
    HANDLE2(FilteredSensorDataBH,
    {
      ALLOC(FrameInfoBH)
      FrameInfoBH& frameInfo = (FrameInfoBH&) *representationBuffer[idFrameInfo];
      const FilteredSensorDataBH& filteredSensorData = (const FilteredSensorDataBH&) *representationBuffer[idFilteredSensorData];
      frameInfo.cycleTime = (float) (filteredSensorData.timeStamp - frameInfo.time) * 0.001f;
      frameInfo.time = filteredSensorData.timeStamp;
    })
    HANDLE2(BehaviorControlOutputBH,
    {
      behaviorControlOutput = (const BehaviorControlOutputBH&) *representationBuffer[idBehaviorControlOutput];
    })
    HANDLE(ObstacleModelBH)
    HANDLE(FilteredJointDataBH)
    HANDLE(CombinedWorldModelBH)
    HANDLE(CameraMatrixBH)
    HANDLE(RobotPerceptBH)
    HANDLE(ImageCoordinateSystemBH)
    HANDLE(LineSpotsBH)
    HANDLE(LocalizationTeamBallBH)
    HANDLE(RobotPoseBH)
    HANDLE(SideConfidenceBH)
    HANDLE(MotionInfoBH)
    HANDLE(GroundTruthWorldStateBH)
    HANDLE(OdometerBH)
    HANDLE(GroundContactStateBH)
    HANDLE(ReceivedSPLStandardMessagesBH)

  case idProcessFinished:
    frameDataComplete = true;
    return true;

  case idStopwatch:
  {
    const int size = message.getMessageSize();
    std::vector<unsigned char> data;
    data.resize(size);
    message.bin.read(&data[0], size);
    Global::getDebugOut().bin.write(&data[0], size);
    Global::getDebugOut().finishMessage(idStopwatch);
    return true;
  }
  case idJPEGImage:
    ALLOC(ImageBH)
    {
      JPEGImage jpegImage;
      message.bin >> jpegImage;
      jpegImage.toImage((ImageBH&) *representationBuffer[idImage]);
    }
    ALLOC(FrameInfoBH)
    ((FrameInfoBH&) *representationBuffer[idFrameInfo]).time = ((ImageBH&) *representationBuffer[idImage]).timeStamp;
    return true;

  default:
    return false;
  }
}

MAKE_MODULE(CognitionLogDataProvider, Cognition Infrastructure)
