/**
* @author Alexis Tsogias
*/

#pragma once

#include "Tools/ModuleBH/Module.h"
#include "Representations/Infrastructure/Thumbnail.h"

MODULE(ThumbnailProvider,
{,
  REQUIRES(ImageBH),
  PROVIDES_WITH_OUTPUT(ThumbnailBH),
  LOADS_PARAMETERS(
  {,
    (unsigned) downScales,
    (bool) grayscale,
  }),
});

class ThumbnailProvider : public ThumbnailProviderBase
{
public:
  void update(ThumbnailBH& thumbnail);

private:
  void shrinkNxN(const ImageBH& srcImage, ThumbnailBH::ThumbnailImage& destImage);
  void shrink8x8SSE(const ImageBH& srcImage, ThumbnailBH::ThumbnailImage& destImage);
  void shrink4x4SSE(const ImageBH& srcImage, ThumbnailBH::ThumbnailImage& destImage);
  void shrinkGrayscaleNxN(const ImageBH& srcImage, ThumbnailBH::ThumbnailImageGrayscale& destImage);
  void shrinkGrayscale8x8SSE(const ImageBH& srcImage, ThumbnailBH::ThumbnailImageGrayscale& destImage);
  void shrinkGrayscale4x4SSE(const ImageBH& srcImage, ThumbnailBH::ThumbnailImageGrayscale& destImage);
};
