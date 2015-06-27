/*
 * @file RobotImage.cpp
 * @author Megan Maher
 * @date July 2015
 *                        -------------------
 *                       |  Robot Detection  |
 *                        -------------------
 * This module is very simple and has many opporunities for improvement.
 * We go through all edges found previously in vision, exlude the ones
 * that are part of hough lines, then look to find the bottom-most edge
 * and the top-most edge in each column of the image.
 *
 * Then, for each bottom edge, if it is not too far down or far up in
 * the image, if more than half the pixels above the edge are white,
 * we consider that there is preliminary evidence in the column.
 *
 * Then, going through the preliminary evidence column by column, if half
 * of the surrounding columns have preliminay evidence, we consider this
 * column to have enough evidence to be considered an obstacle.
 *
 * After going through all this evidence, we build a box that constricts
 * the obstacle, and update the "obstacleBox" that exists in the Vision
 * Module and is then passed out as a protobuf.
 *
 * Many opportunities for further work.
 *      - Dealing with field crosses and center circle in image
 *      - Doesn't detect robot feet
 *      - obstacleBox is not precise at all:
 *              could be using parts of field cross / field lines
 */

#include "RobotImage.h"

namespace man {
namespace vision {

RobotImage::RobotImage(int wd_, int ht_) //: robotVisionOut(base())
{
    img_wd = wd_;
    img_ht = ht_;
    std::cout<<"[ ROBOT IMAGE] width = "<<wd_<<", height = "<<ht_<<std::endl;
}

// Run every frame from VisionModule.cpp
void RobotImage::updateVisionObstacle(ImageLiteU8 whiteImage, EdgeList& edges,
                                      int* obstacleBox)
{
    int bottom[img_wd];      // not used
    int top[img_wd];         // not used
    int maxbottom[img_wd];   // lowest edge that points up
    int mintop[img_wd];      // highest edge that points down
    bool prelim[img_wd];     // is there evidence of an obstacle
                             //    after our first round of checks?
    bool evidence[img_wd];   // is there evidence of an obstacle in this column?

    initAccumulators(bottom, top, maxbottom, mintop,
                     prelim, evidence, obstacleBox);

    // printArray(maxbottom, img_wd, "MAX BOTTOM WAY BEFORE");

    // Go through edges and determine which are "topmost" and "bottommost" ones
    getBottomAndTopEdges(edges, mintop, maxbottom);

    // printArray(maxbottom, img_wd, "MAX BOTTOM WAY AFTER");

    // Determine if a column has enough evidence of an obstacle
    determinePrelimEvidence(whiteImage, img_wd, img_ht, maxbottom, prelim);

    // printArray(prelim, img_wd, "PRELIM");

    // Determine if we actually think there is an obstacle in this column
    //           based on cells in surrounding columns
    determineFinalEvidence(img_wd, img_ht, maxbottom, prelim, evidence, obstacleBox);

    // std::cout<<"CHecking for visual obstacle"<<std::endl;

    // printArray(evidence, img_wd, "EVIDENCE");

    obstacleBox[0] -= img_ht/2;
    obstacleBox[1] -= img_ht/2;
    obstacleBox[2] -= img_wd/2;
    obstacleBox[3] -= img_wd/2;
}

void RobotImage::initAccumulators(int* bottom, int* top, int* maxbottom, int* mintop,
                                  bool* prelim, bool* evidence, int* obstacleBox)
{
    for (int i = 0; i < img_wd; i++) {
        bottom[i] = 0;
        top[i] = img_ht;
        maxbottom[i] = img_ht;
        mintop[i] = 0;
        prelim[i] = false;
        evidence[i] = false;
    }

    // {| top | bottom | left | right |} coordinate values
    obstacleBox[0] = -1;
    obstacleBox[1] = img_ht;
    obstacleBox[2] = img_wd;
    obstacleBox[3] = 0;
}

void RobotImage::printArray(int* array, int size, std::string name)
{
    std::cout<<name<<": ["<<array[0]<<"]";
    for (int i = 1; i < size; i++) {
        std::cout<<"  ["<<array[i]<<"]";
    }
    std::cout<<std::endl;
}

void RobotImage::printArray(bool* array, int size, std::string name)
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

void RobotImage::getBottomAndTopEdges(EdgeList& edges, int* mintop, int* maxbottom)
{
    // Get edges from vision
    AngleBinsIterator<Edge> abi(edges);
    for (Edge* e = *abi; e; e = *++abi){
        // If we are part of a hough line, we are not a robot edge
        if (e->memberOf()) { continue; }

        int x = e->x() + img_wd/2;
        int y = e->y() + img_ht/2;
        // int mag = e->mag();         // magnitude - could be useful?
        int ang = e->angle();

        // don't want to get too high or low in the image
        if (y < 15 || y > (img_ht - 30) ) { continue; }

        if (ang < 128 && y < maxbottom[x]) {
            maxbottom[x] = y;
        } else if ( ang > 128 && y > mintop[x]) {
            mintop[x] = y;
        }
    }
}

void RobotImage::determinePrelimEvidence(ImageLiteU8 whiteImage, int img_wd,
                                         int img_ht, int* maxbottom, bool* prelim)
{
    // std::cout<<"Determine prelim evidence"<<std::endl;
    for (int i = 0; i < img_wd; i++) {
        // std::cout<<"inside loop "<<i<<std::endl;
        // if there is an upward pointing edge in this column
        if (maxbottom[i] < img_ht && maxbottom[i] >= 0) {
            // this is the key business, look for mostly white between edge and top
            int w = 0;
            for (int j = maxbottom[i]; j < img_ht; j++) { // loop from edge to top
                // std::cout<<"tinier loop "<<j<<std::endl;
                if (*whiteImage.pixelAddr(i-img_wd/2,j-img_ht/2) > 128) {
                    w++;
                }
            }
            // std::cout<<"White: "<<w<<std::endl;
            if (w > (img_ht - maxbottom[i]) / 2) {
                prelim[i] = true;
            }
        }
    }
}

void RobotImage::determineFinalEvidence(int img_wd, int img_ht, int* maxbottom,
                                        bool* prelim, bool* evidence, int *obstacleBox)
{
    // std::cout<<"Determine final evidence"<<std::endl;

    // std::cout<<"Obstacle box:"<<obstacleBox[0]<<", "<<obstacleBox[1]<<", "
    //         <<obstacleBox[2]<<", "<<obstacleBox[3]<<std::endl;

    // determine if we actually think there is an obstacle in this column
    // based on cells around it
    for (int i = 0; i < img_wd; i++) {
        // we can use previous evidence to determine this evidence
        if ( ( i != 0 && i < img_wd-3 ) && ( evidence[i-1] && prelim[i+3] ) ) {
            evidence[i] = true;
        } else {
            int total = 0, count = 0;
            for (int j = i - 3; j <= i + 3; j++) {
                if (j >= 0 && j < img_wd) {
                    total++;
                    if (prelim[j]) { count++; }
                }
            }
            // if more than half of me + my surrounding pixels are true, I am true
            if (2 * count > total) {
                evidence[i] = true;
            }
        }

        // if (evidence[i]) { std::cout<<"EVIDENCE AT "<<i<<std::endl; }

        // update box params
        if (evidence[i] && maxbottom[i] >=0 && maxbottom[i] < img_ht) {
            // TOP:
            // if (maxbottom[i] < &obstacleBox[0]) { &obstacleBox[0] = maxbottom[i]; }

            // BOTTOM:
            if (maxbottom[i] < obstacleBox[1]) { obstacleBox[1] = maxbottom[i]; }

            // LEFT: This will only get set once, for first evidence found
            if (i < obstacleBox[2])   { obstacleBox[2] = i; }

            // RIGHT:
            if (i > obstacleBox[3])  { obstacleBox[3] = i; }

            // std::cout<<"Obstacle box:"<<obstacleBox[0]<<", "<<obstacleBox[1]<<", "
            //          <<obstacleBox[2]<<", "<<obstacleBox[3]<<std::endl;
        }
    }
}

} //namespace vision
} //namespace man
