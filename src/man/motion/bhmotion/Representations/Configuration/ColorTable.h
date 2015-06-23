/**
 * @file ColorTable.h
 * The file declares a class that represents the tabularized color calibration.
 * @author: marcel
 */

#pragma once

#include "Representations/Infrastructure/Image.h"
#include "ColorCalibration.h"

class ColorTableBH : public Streamable
{
public:
    struct ColorsBH
    {
        unsigned char colors;

        ColorsBH() : colors(0) {}
        ColorsBH(unsigned char colorsBH) : colors(colors) {}
        ColorsBH(ColorClassesBH::Color color)
        : colors(color == ColorClassesBH::none ? 0 : (unsigned char) (1 << (color - 1))) {}

        bool is(ColorClassesBH::Color color) const
        {
            return (color == ColorClassesBH::none && !colors) ||
                         (1 << (color - 1) & colors) != 0;
        }
    };

    void fromColorCalibration(const ColorCalibrationBH& colorCalibration, ColorCalibrationBH& prevCalibration);

    const ColorsBH& operator[](const ImageBH::Pixel& pixel) const
    {
        return colorTable[pixel.y >> 3][pixel.cb][pixel.cr];
    }

private:
    ColorsBH colorTable[32][256][256];

    void update(const ColorCalibrationBH::HSIRanges& ranges, unsigned char color);
    void update(const ColorCalibrationBH::WhiteThresholds& thresholds, unsigned char color);

    virtual void serialize(In* in, Out* out);
};

inline Out& operator<<(Out& stream, const ColorTableBH::ColorsBH& color)
{
    STREAM_REGISTER_BEGIN_EXT(color);
    STREAM_EXT(stream, color.colors);
    STREAM_REGISTER_FINISH;
    return stream;
}

inline In& operator>>(In& stream, ColorTableBH::ColorsBH& color)
{
    STREAM_REGISTER_BEGIN_EXT(color);
    STREAM_EXT(stream, color.colors);
    STREAM_REGISTER_FINISH;
    return stream;
}
