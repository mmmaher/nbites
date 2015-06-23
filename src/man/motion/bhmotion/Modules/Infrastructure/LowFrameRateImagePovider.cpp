/**
 * @file Modules/Infrastructure/LowFrameRateImageProvider.h
 * This file implements a module that provides an image that is only rarely updated.
 * The image is intended for logging purposes.
 * @author Arne Böckmann
 */

#include "LowFrameRateImageProvider.h"

void LowFrameRateImageProvider::update(LowFrameRateImageBH& lowFrameRateImage)
{
    if(theFrameInfoBH.getTimeSince(lastUpdateTime) >= 1000 / frameRate)
    { // Generate new image
        lastUpdateTime = theFrameInfoBH.time;
        updateImage(lowFrameRateImage);
        storeNextImage = true; // Store next image as well to make sure to get both upper and lower cam images
    }
    else if(storeNextImage)
    {
        updateImage(lowFrameRateImage);
        storeNextImage = false;
    }
    else
        lowFrameRateImage.imageUpdated = false;
}

void LowFrameRateImageProvider::updateImage(LowFrameRateImageBH& lfrImage) const
{
    lfrImage.image.setImage(const_cast<ImageBH::Pixel*>(theImageBH[0]));
    lfrImage.image.setResolution(theImageBH.width, theImageBH.height, theImageBH.isFullSize);
    lfrImage.image.timeStamp = theImageBH.timeStamp;
    lfrImage.imageUpdated = true;
}

MAKE_MODULE(LowFrameRateImageProvider, Cognition Infrastructure)