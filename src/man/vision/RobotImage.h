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
    // const int minLength = 35;

public:
    RobotImage(int wd_, int ht_);
    void updateVisionObstacle(ImageLiteU8 whiteImage, EdgeList& edges, int* obstacleBox);

protected:
    int getBoxTop();
    int getBoxBottom();
    int getBoxLeft();
    int getBoxRight();

private:
    void initAccumulators(int* obstacleBox);
    void getBottomAndTopEdges(EdgeList& edges);
    void determinePrelimEvidence(ImageLiteU8 whiteImage);
    void determineFinalEvidence(int* obstacleBox);

    void printArray(int* array, int size, std::string name);
    void printArray(bool* array, int size, std::string name);

    int img_wd;
    int img_ht;

    int* bottom;      // not used
    int* top;         // not used
    int* maxbottom;   // lowest edge that points up
    int* mintop;      // highest edge that points down
    bool* prelim;     // is there evidence of an obstacle
                             //    after our first round of checks?
    bool* evidence;   // is there evidence of an obstacle in this column?
};

} // namespace vision
} // namespace man
