/*
 *  -----------------------
 * |  Robot Detection 2015 |
 *  -----------------------
 * @file RobotObstacle.cpp
 * @author Megan Maher
 * @created July 2015
 * @modified July 2015
 */

#include "RobotObstacle.h"

namespace man {
namespace vision {

RobotObstacle::RobotObstacle(int wd_, int ht_)
{
    std::cout<<"[ ROBOT IMAGE] width = "<<wd_<<", height = "<<ht_<<std::endl;
    img_wd = wd_;
    img_ht = ht_;

    bottom = new int[img_wd];
    top = new int[img_wd];
    maxbottom = new int[img_wd];
    mintop = new int[img_wd];
}

void RobotObstacle::printArray(int* array, int size, std::string name)
{
    std::cout<<name<<": ["<<array[0]<<"]";
    for (int i = 1; i < size; i++) {
        std::cout<<"  ["<<array[i]<<"]";
    }
    std::cout<<std::endl;
}

void RobotObstacle::printArray(bool* array, int size, std::string name)
{
    std::cout<<name<<": ["<<array[0]<<"]";
    for (int i = 1; i < size; i++) {
        if (array[i]){
            std::cout<<"  ["<<i<<"]";
            continue;
        }
        std::cout<<"  [0]";
    }
    std::cout<<std::endl;
}

// @DEBUGGING
// void RobotObstacle::updateVisionObstacle(ImageLiteU8 whiteImage, EdgeList& edges,
//                                       int* obstacleBox, int* whiteBools, int* edgeBools)

// Run every frame from VisionModule.cpp
void RobotObstacle::updateVisionObstacle(ImageLiteU8 whiteImage, EdgeList& edges,
                                      int* obstacleBox)
{
    initAccumulators(obstacleBox);

    // Go through edges and determine which are "topmost" and "bottommost" ones
    getBottomAndTopEdges(edges);

    // Find constricting box of obstacle in image coordinates
    findObstacle(whiteImage, obstacleBox);

    // @DEBUGGING
    // for (int i = 0; i < img_wd*img_ht; i++) {
    //     edgeBools[i] = 0;
    // }
    // getBottomAndTopEdges(edges, edgeBools);
    // printArray(maxbottom, img_wd, "MAX BOTTOM AFTER");
    // findObstacle(whiteImage, whiteBools, obstacleBox);
}

void RobotObstacle::initAccumulators(int* obstacleBox)
{
    for (int i = 0; i < img_wd; i++) {
        bottom[i] = img_ht;
        top[i] = 0;
        maxbottom[i] = 0;
        mintop[i] = img_ht;
    }

    // {| top | bottom | left | right |} coordinate values of box
    obstacleBox[0] = -1;
    obstacleBox[1] = -1;
    obstacleBox[2] = -1;
    obstacleBox[3] = -1;
}

void RobotObstacle::getBottomAndTopEdges(EdgeList& edges)
{
    // Get edges from vision
    AngleBinsIterator<Edge> abi(edges);
    for (Edge* e = *abi; e; e = *++abi){
        // If we are part of a hough line, we are not a robot edge
        if (e->memberOf()) { continue; }

        int x = e->x() + img_wd/2;
        int y = img_ht/2 - e->y();
        int ang = e->angle();
        // int mag = e->mag();      // magnitude - could be useful later?

        //@DEBUGGING
        // edgeBools[img_wd * y + x] = 1;

        // don't want to get too high or low in the image
        if (y < BARRIER_TOP || y > img_ht - BARRIER_BOT ) { continue; }

        // binary angles, so 128 = pi radians
        if (ang < 128 && y > maxbottom[x]) {
            maxbottom[x] = y;
        } else if ( ang > 128 && y < mintop[x]) {
            mintop[x] = y;
        }
    }
}

void RobotObstacle::findObstacle(ImageLiteU8 whiteImage, int* obstacleBox)
{
    int maxLength = 0;          // max run length we've found so far
    int maxStart = -1;          // start of the max run
    int maxBot = 0;             // lowest point of the max run
    int currLength = 0;         // length of our current run
    int currStart = 0;          // start of first run, start at first index
    int currBot = 0;            // lowest edge of our current run
    int blankCounter = 0;       // how many columns without evidence in a row we've found

    // Loop through each column to find a run of columns with evidence of an obstacle
    for (int i = 0; i < img_wd; i++) {
        int w = 0;
        for (int j = 0; j < maxbottom[i]; j++) {
            // this is the key business, look for mostly white between edge and top
            if (*(whiteImage.pixelAddr(i,j)) > WHITE_CONFIDENCE_THRESH) {
                w++;
            }
        }
        if (w > maxbottom[i] / 2) {
            // there is enough evidence for this column!
            if (maxbottom[i] > currBot) { currBot = maxbottom[i]; }
            if (currStart == -1) { currStart = i; }
            blankCounter = 0;
            currLength++;
        } else if (currStart != -1) {   // ignore when we haven't started a run
            if (blankCounter >= MAX_DIST) {
                // no evidence and blank counter too high, reset params
                currLength -= blankCounter; // get rid of blanks we counted prematurely
                if (currLength > maxLength) {
                    maxLength = currLength;
                    maxStart = currStart;
                    maxBot = currBot;
                }
                currLength = 0;
                currStart = -1;
                currBot = 0;
                blankCounter = 0;
            } else {
                // just a blank, let's increment the blank counter and continue
                blankCounter++;
                currLength++;
            }
        }
        // @DEBUGGING
        // std::cout<<"    MAX: l = "<<maxLength<<", s = "<<maxStart<<", b = "<<maxBot<<std::endl;
        // std::cout<<"    CURR: l = "<<currLength<<", s = "<<currStart<<", b = "<<currBot<<std::endl;
    }

    // check to see if we ended with our maximum length:
    if (currLength > maxLength) {
        maxLength = currLength;
        maxStart = currStart;
        maxBot = currBot;
    }

    if (maxLength > MIN_LENGTH) {
        // now update obstacle box
        obstacleBox[1] = maxBot;                // bottom
        obstacleBox[2] = maxStart;              // left
        obstacleBox[3] = maxStart + maxLength;  // right
    }
}

} //namespace vision
} //namespace man
