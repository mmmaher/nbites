/**
 * @file Modules/Infrastructure/LowFrameRateImageProvider.h
 * This file declares a module that provides an image that is only rarely updated.
 * The image is intended for logging purposes.
 * @author Arne Böckmann
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/LowFrameRateImage.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"

MODULE(LowFrameRateImageProvider,
{,
  REQUIRES(ImageBH),
  REQUIRES(CameraInfoBH),
  REQUIRES(FrameInfoBH),
  PROVIDES(LowFrameRateImageBH),
  LOADS_PARAMETERS(
  {,
    (int) frameRate, /**< Frames per second. */
  }),
});

class LowFrameRateImageProvider : public LowFrameRateImageProviderBase
{
public:
  LowFrameRateImageProvider() : lastUpdateTime(0), storeNextImage(false) {}
  void update(LowFrameRateImageBH& image);

private:
  void updateImage(LowFrameRateImageBH& lfrImage) const;

  unsigned lastUpdateTime; /**< Time of last update. */
  bool storeNextImage;
};