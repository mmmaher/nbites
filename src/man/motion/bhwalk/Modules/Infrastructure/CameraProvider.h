/**
* @file CameraProvider.h
* This file declares a module that provides camera images.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

class NaoCamera;

#include "Tools/ModuleBH/Module.h"
#include "Platform/Camera.h"
#include "Representations/Infrastructure/CameraSettings.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/Infrastructure/CameraResolution.h"

// MODULE(CameraProvider)
//   REQUIRES(CameraSettingsBH)
//   REQUIRES(ImageBH)
//   PROVIDES_WITH_OUTPUT(ImageBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfoBH)
//   PROVIDES_WITH_MODIFY(CognitionFrameInfoBH)
//   PROVIDES_WITH_MODIFY_AND_OUTPUT(CameraInfoBH)
// END_MODULE

MODULE(CameraProvider,
{,
  USES(CameraIntrinsicsNextBH),
  USES(CameraResolutionRequestBH),
  REQUIRES(ImageBH),
  PROVIDES_WITH_OUTPUT(ImageBH),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfoBH),
  PROVIDES_WITH_OUTPUT(CameraInfoBH),
  PROVIDES(CameraInfoFullResBH),
  PROVIDES(CameraSettingsBH),
  PROVIDES_WITH_MODIFY(CameraIntrinsicsBH),
  PROVIDES(CameraResolutionBH),
});

class CameraProvider : public CameraProviderBase
{
private:
  static PROCESS_WIDE_STORAGE(CameraProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  NaoCamera* upperCamera = nullptr;
  NaoCamera* lowerCamera = nullptr;
  NaoCamera* currentImageCamera = nullptr;
  CameraInfoBH upperCameraInfo;
  CameraInfoBH lowerCameraInfo;
  CameraSettingsBH upperCameraSettings;
  CameraSettingsBH lowerCameraSettings;
  CameraIntrinsicsBH cameraIntrinsics;
  CameraResolutionBH cameraResolution;
  float cycleTime;
#ifdef CAMERA_INCLUDED
  unsigned int imageTimeStamp;
  unsigned int otherImageTimeStamp;
  unsigned int lastImageTimeStamp;
  unsigned long long lastImageTimeStampLL;
#endif

public:
  /**
  * Default constructor.
  */
  CameraProvider();

  /**
  * Destructor.
  */
  ~CameraProvider();

  /**
  * The method returns whether a new image is available.
  * @return Is an new image available?
  */
  static bool isFrameDataComplete();

  /**
  * The method waits for a new image.
  */
  static void waitForFrameData();
  void waitForFrameData2();
  void setUpCameras();

private:
  void update(ImageBH& image);
  void update(FrameInfoBH& frameInfo);
  void update(CameraInfoBH& cameraInfo);
  void update(CameraInfoFullResBH& cameraInfoFullRes);
  void update(CameraSettingsBH& cameraSettings);
  void update(CameraIntrinsicsBH& cameraIntrinsics);
  void update(CameraResolutionBH& cameraResolution);

  bool readCameraSettings();
  bool readCameraIntrinsics();
  bool readCameraResolution();

  bool processResolutionRequest();

  void setupCameras();
};
