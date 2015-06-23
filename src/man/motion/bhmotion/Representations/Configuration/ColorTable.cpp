/**
 * @file ColorTable.h
 * The file implements a class that represents the tabularized color calibration.
 * @author: marcel
 */

#include "ColorTable.h"
#include "ColorCalibration.h"
#include "Platform/BHAssert.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "snappy-c.h"

/**
 * Tables that map YCbCr color values to HSI or RB color values.
 * The resolution of this table is the one used by the ColorCalibration class.
 */
static class ColorSpaceMapperBH
{
public:
  struct RB
  {
    unsigned char r;
    unsigned char b;
    unsigned short rb;
  };

  ImageBH::Pixel hsi[32][256][256];
  RB rb[32][256][256];

  ColorSpaceMapperBH()
  {
    ImageBH::Pixel* p = &hsi[0][0][0];
    RB* q = &rb[0][0][0];
    unsigned char g;
    for(int y = 0; y < 256; y += 8)
      for(int cb = 0; cb < 256; ++cb)
        for(int cr = 0; cr < 256; ++cr, ++p, ++q)
        {
          ColorModelConversionsBH::fromYCbCrToHSI((unsigned char) y,
                                                (unsigned char) cb,
                                                (unsigned char) cr,
                                                p->h, p->s, p->i);
          ColorModelConversionsBH::fromYCbCrToRGB((unsigned char) y,
                                                (unsigned char) cb,
                                                (unsigned char) cr,
                                                q->r, g, q->b);
          q->rb = (unsigned short) q->r + q->b;
        }
  }
} colorSpaceMapper;

void ColorTableBH::fromColorCalibration(const ColorCalibrationBH& colorCalibration, ColorCalibrationBH& prevCalibration)
{
  bool greenChanged = colorCalibration.ranges[ColorClassesBH::green - 2] != prevCalibration.ranges[ColorClassesBH::green - 2];
  for(unsigned i = 0; i < ColorClassesBH::numOfColors - 2; ++i)
    if(colorCalibration.ranges[i] != prevCalibration.ranges[i])
    {
      update(colorCalibration.ranges[i], (unsigned char) (1 << (i + 1)));
      prevCalibration.ranges[i] = colorCalibration.ranges[i];
    }

  if(colorCalibration.white != prevCalibration.white || greenChanged)
  {
    update(colorCalibration.white, 1 << (ColorClassesBH::white - 1));
    prevCalibration.white = colorCalibration.white;
  }
}

void ColorTableBH::update(const ColorCalibrationBH::HSIRanges& ranges, unsigned char color)
{
  Colors* dest = &colorTable[0][0][0];
  for(const ImageBH::Pixel* src = &colorSpaceMapper.hsi[0][0][0],
                        * end = &colorSpaceMapper.hsi[32][0][0];
      src < end; ++src, ++dest)
    if(ranges.hue.isInside(src->h) &&
       ranges.saturation.isInside(src->s) &&
       ranges.intensity.isInside(src->i))
      dest->colors |= color;
    else
      dest->colors &= ~color;
}

void ColorTableBH::update(const ColorCalibrationBH::WhiteThresholds& thresholds, unsigned char color)
{
  Colors* dest = &colorTable[0][0][0];
  for(const ColorSpaceMapperBH::RB* src = &colorSpaceMapper.rb[0][0][0],
                                * end = &colorSpaceMapper.rb[32][0][0];
      src < end; ++src, ++dest)
    if(src->r >= thresholds.minR &&
       src->b >= thresholds.minB &&
       src->rb >= thresholds.minRB &&
       !(dest->colors & 1 << (ColorClassesBH::green - 1)))
      dest->colors |= color;
    else
      dest->colors &= ~color;
}

void ColorTableBH::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  size_t ctUncompressedSize = sizeof(colorTable);
  size_t ctCompressedSize = 0;
  std::vector<char> ctCompressed;

  if(out)
  {
    ctCompressedSize = snappy_max_compressed_length(ctUncompressedSize);
    ctCompressed.resize(ctCompressedSize);
    VERIFY(snappy_compress((char*) &colorTable[0][0][0], ctUncompressedSize, ctCompressed.data(), &ctCompressedSize) == SNAPPY_OK);
    out->write(&ctCompressedSize, sizeof(int));
    out->write(ctCompressed.data(), ctCompressedSize);
  }
  else
  {
    in->read(&ctCompressedSize, sizeof(int));
    ctCompressed.resize(ctCompressedSize);
    in->read(ctCompressed.data(), ctCompressedSize);
    VERIFY(snappy_uncompress(ctCompressed.data(), ctCompressedSize, (char*) &colorTable[0][0][0], &ctUncompressedSize) == SNAPPY_OK);
    ASSERT(ctUncompressedSize == sizeof(colorTable));
  }
  STREAM_REGISTER_FINISH;
}
