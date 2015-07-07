// **************************
// *                        *
// *  Robot Detection 2015  *
// *                        *
// **************************

#pragma once

// #include <stdlib.h>
#include <string.h>
// #include <math.h>
#include "Hough.h"

//#include "RoboGrams.h"
// #include "NBMath.h"

namespace man {
namespace vision {

class RobotImage
{
    const int MAX_DIST = 4; // max columns we can have "empty" in a row for a run
    const int MIN_LENGTH = 35; // min number for a run
    const int BARRIER_TOP = 15; // amount on top of image we don't want to process
    const int BARRIER_BOT = 30; // amount on bot of image we don't want to process

public:
    RobotImage(int wd_, int ht_);
    void updateVisionObstacle(ImageLiteU8 whiteImage, EdgeList& edges, int* obstacleBox,
                              int* whiteBools, int* edgeBools);

private:
    void initAccumulators(int* obstacleBox);
    void getBottomAndTopEdges(EdgeList& edges, int* edgeBools);
    void findObstacle(ImageLiteU8 whiteImage, int* whiteBools, int* obstacleBox);

    void printArray(int* array, int size, std::string name);
    void printArray(bool* array, int size, std::string name);

    int img_wd;
    int img_ht;

    int* bottom;      // highest edge that points up (not used currently)
    int* top;         // lowest edge that points down (not used currently)
    int* maxbottom;   // lowest edge that points up
    int* mintop;      // highest edge that points down (not currently used)
};

} // namespace vision
} // namespace man
