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
    void initAccumulators(int* bottom, int* top, int* maxbottom, int* mintop,
                          bool* prelim, bool* evidence, int* obstacleBox);
    void getBottomAndTopEdges(EdgeList& edges, int* mintop, int* maxbottom);
    void determinePrelimEvidence(ImageLiteU8 whiteImage, int img_wd,
                                 int img_ht, int* maxbottom, bool* prelim);
    void determineFinalEvidence(int img_wd, int img_ht, int* maxbottom,
                                bool* prelim, bool* evidence, int* obstacleBox);

    void printArray(int* array, int size, std::string name);
    void printArray(bool* array, int size, std::string name);

    int img_wd;
    int img_ht;
};

} // namespace vision
} // namespace man
