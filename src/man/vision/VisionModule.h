#pragma once

#include "RoboGrams.h"
#include "Camera.h"
#include "Images.h"
#include "PMotion.pb.h"
#include "FrontEnd.h"
#include "Edge.h"
#include "Hough.h"
#include "Kinematics.h"
#include "Homography.h"
#include "Field.h"

#define RICH_LOGGING

namespace man {
namespace vision {

class VisionModule : public portals::Module {
public:
    VisionModule();
    virtual ~VisionModule();

    portals::InPortal<messages::YUVImage> topIn;
    portals::InPortal<messages::YUVImage> bottomIn;
    portals::InPortal<messages::JointAngles> jointsIn;

    ImageFrontEnd* getFrontEnd(bool topCamera = true) const { return frontEnd[!topCamera]; }
    EdgeList* getEdges(bool topCamera = true) const { return edges[!topCamera]; }
    HoughLineList* getLines(bool topCamera = true) const { return houghLines[!topCamera]; }
	DebugImage* getDebugImage(bool topCamera = true) const { return debugImage[!topCamera]; }

protected:
    virtual void run_();

private:
    Colors* colorParams[2];
    ImageFrontEnd* frontEnd[2];
    EdgeDetector* edgeDetector[2];
    EdgeList* edges[2];
    HoughLineList* houghLines[2];
    HoughSpace* hough[2];
    Kinematics* kinematics[2];
    FieldHomography* homography[2];
    FieldLineList* fieldLines[2];
	DebugImage* debugImage[2];
	Field* field;
};

}
}
