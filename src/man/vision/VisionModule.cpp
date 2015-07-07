#include "VisionModule.h"
#include "Edge.h"
#include "HighResTimer.h"
#include "NBMath.h"
#include "../control/control.h"
#include "../log/logging.h"

#include <fstream>
#include <iostream>
#include <fstream>
#include <chrono>

#include "Profiler.h"
#include "DebugConfig.h"
//#include "PostDetector.h"

namespace man {
namespace vision {

VisionModule::VisionModule(int wd, int ht, std::string robotName)
    : Module(),
      topIn(),
      bottomIn(),
      jointsIn(),
      linesOut(base()),
      cornersOut(base()),
      ballOut(base()),
      robotObstacleOut(base()),
      ballOn(false),
      ballOnCount(0),
      ballOffCount(0),
      centCircOut(base()),
      centerCircleDetected(false),
      blackStar_(false)
{
    // this is in here because this module is usually run with the "top" parameters
    // and with the tool we are just running the bottom parameters
#ifdef OFFLINE
    wd = 640;
    ht = 480;
#endif

    std:: string colorPath, calibrationPath;
    #ifdef OFFLINE
        colorPath =  calibrationPath = std::string(getenv("NBITES_DIR"));
        colorPath += "/src/man/config/colorParams.txt";
        calibrationPath += "/src/man/config/calibrationParams.txt";
    #else
        colorPath = "/home/nao/nbites/Config/colorParams.txt";
        calibrationPath = "/home/nao/nbites/Config/calibrationParams.txt";
    #endif

    // Get SExpr from string
    nblog::SExpr* colors = nblog::SExpr::read(getStringFromTxtFile(colorPath));
    calibrationLisp = nblog::SExpr::read(getStringFromTxtFile(calibrationPath));

    // Set module pointers for top then bottom images
    // NOTE Constructed on heap because some of the objects below do
    //      not have default constructors, all class members must be initialized
    //      after the initializer list is run, which requires calling default
    //      constructors in the case of C-style arrays, limitation theoretically
    //      removed in C++11.
    for (int i = 0; i < 2; i++) {
        colorParams[i] = getColorsFromLisp(colors, i);
        frontEnd[i] = new ImageFrontEnd();
        edgeDetector[i] = new EdgeDetector();
        edges[i] = new EdgeList(32000);
        rejectedEdges[i] = new EdgeList(32000);
        houghLines[i] = new HoughLineList(128);
        calibrationParams[i] = new CalibrationParams();
        kinematics[i] = new Kinematics(i == 0);
        homography[i] = new FieldHomography(i == 0);
        fieldLines[i] = new FieldLineList();
        if (i == 0) {
            field[i] = new Field(wd / 2, ht / 2, homography[i]);
        } else {
            field[i] = new Field(wd / 4, ht / 4, homography[i]);
        }
#ifdef OFFLINE
		// Get the appropriate amount of space for the Debug Image
		if (i == 0) {
			debugSpace[0] = (uint8_t *)malloc(wd * ht * sizeof(uint8_t));
		} else {
			debugSpace[1] = (uint8_t *)malloc((wd / 2) * (ht / 2) * sizeof(uint8_t));
		}
#else
		// don't get any space if we're running on the robot
		debugSpace[i] = NULL;
#endif
        std::cout<<"VISION WID: "<<wd<<" "<<ht<<std::endl;
		// Construct the lightweight debug images that know where the space is
		if (i == 0) {
			debugImage[i] = new DebugImage(wd, ht, debugSpace[0]);
			debugImage[i]->reset();
		} else {
			debugImage[i] = new DebugImage(wd / 2, ht / 2, debugSpace[1]);
			debugImage[i]->reset();
		}

        ballDetector[i] = new BallDetector(homography[i], i == 0);
        boxDetector[i] = new GoalboxDetector();
        centerCircleDetector[i] = new CenterCircleDetector();

        if (i == 0) {
          hough[i] = new HoughSpace(wd / 2, ht / 2);
          cornerDetector[i] = new CornerDetector(wd / 2, ht / 2);
        } else {
          hough[i] = new HoughSpace(wd / 4, ht / 4);
          cornerDetector[i] = new CornerDetector(wd / 4, ht / 4);
        }

        bool fast = true;
        frontEnd[i]->fast(fast);
        hough[i]->fast(fast);

#ifdef OFFLINE
        // Here is an example of how to get access to the debug space. In this case the
        // field class only runs on the top image so it only needs that one
        field[i]->setDebugImage(debugImage[i]);
#endif
    }
    robotImageObstacle = new RobotImage(wd / 4, ht / 4);
    setCalibrationParams(robotName);
}

VisionModule::~VisionModule()
{
    for (int i = 0; i < 2; i++) {
        delete colorParams[i];
        delete frontEnd[i];
        delete edgeDetector[i];
        delete edges[i];
        delete rejectedEdges[i];
        delete houghLines[i];
        delete hough[i];
        delete calibrationParams[i];
        delete kinematics[i];
        delete homography[i];
        delete fieldLines[i];
		delete debugImage[i];
		delete debugSpace[i];
        delete boxDetector[i];
        delete cornerDetector[i];
        delete centerCircleDetector[i];
        delete ballDetector[i];
        delete field[i];
    }
}

// TODO use horizon on top image
void VisionModule::run_()
{
    // Get messages from inPortals
    topIn.latch();
    bottomIn.latch();
    jointsIn.latch();
    inertsIn.latch();

    // Setup
    std::vector<const messages::YUVImage*> images { &topIn.message(),
                                                    &bottomIn.message() };

    bool ballDetected = false;
    centerCircleDetected = false;

    // Loop over top and bottom image and run line detection system
    for (int i = 0; i < images.size(); i++) {

        // Get image
        const messages::YUVImage* image = images[i];

        // Construct YuvLite object for use in vision system
        YuvLite yuvLite(image->width() / 4,
                        image->height() / 2,
                        image->rowPitch(),
                        image->pixelAddress(0, 0));

        // Run front end
        frontEnd[i]->run(yuvLite, colorParams[i]);
        ImageLiteU16 yImage(frontEnd[i]->yImage());
        ImageLiteU8 whiteImage(frontEnd[i]->whiteImage());
        ImageLiteU8 greenImage(frontEnd[i]->greenImage());
        ImageLiteU8 orangeImage(frontEnd[i]->orangeImage());

        // Calculate kinematics and adjust homography
        if (jointsIn.message().has_head_yaw()) {
            kinematics[i]->joints(jointsIn.message());
            homography[i]->wx0(kinematics[i]->wx0());
            homography[i]->wy0(kinematics[i]->wy0());
            homography[i]->wz0(kinematics[i]->wz0());
            homography[i]->roll(calibrationParams[i]->getRoll());
            homography[i]->tilt(kinematics[i]->tilt() + calibrationParams[i]->getTilt());
#ifndef OFFLINE
            homography[i]->azimuth(kinematics[i]->azimuth());
#endif
        }

        // Approximate brightness gradient
        edgeDetector[i]->gradient(yImage);

        // field needs the color images
        field[i]->setImages(frontEnd[0]->whiteImage(), frontEnd[0]->greenImage(),
                            frontEnd[0]->orangeImage());

		// only calculate the field in the top camera
		if (i ==0) {
            GeoLine horizon = homography[0]->horizon(image->width() / 2);
			double x1, x2, y1, y2;
			horizon.endPoints(x1, y1, x2, y2);
			int hor = static_cast<int>(y1);
			hor = image->height() / 4 - hor;
			field[0]->findGreenHorizon(hor, 0.0f);
		}

        // Run edge detection
        edgeDetector[i]->edgeDetect(greenImage, *(edges[i]));

        // Run hough line detection
        hough[i]->run(*(edges[i]), *(rejectedEdges[i]), *(houghLines[i]));

        // Find world coordinates for hough lines
        houghLines[i]->mapToField(*(homography[i]));

        // Find world coordinates for rejected edges
        // rejectedEdges[i]->mapToField(*(homography[i]));

        // Detect center circle on top
        // if (!i) centerCircleDetected = centerCircleDetector[i]->detectCenterCircle(*(rejectedEdges[i]));

        // Pair hough lines to field lines
        fieldLines[i]->find(*(houghLines[i]), blackStar());

        // Classify field lines
        fieldLines[i]->classify(*(boxDetector[i]), *(cornerDetector[i]));

        ballDetected |= ballDetector[i]->findBall(orangeImage, kinematics[i]->wz0());

#ifdef USE_LOGGING
        logImage(i);
        // if (getenv("LOG_THIS") != NULL) {
        //     if (strcmp(getenv("LOG_THIS"), std::string("top").c_str()) == 0) {
        //         logImage(0);
        //         setenv("LOG_THIS", "false", 1);
        //         std::cerr << "pCal logging top log\n";
        //     } else if (strcmp(getenv("LOG_THIS"), std::string("bottom").c_str()) == 0) {
        //         logImage(1);
        //         setenv("LOG_THIS", "false", 1);
        //         std::cerr << "pCal logging bot log\n";
        //     }// else
        //        // std::cerr << "N ";
        // } else {
        //     logImage(0);
        //     logImage(1);
        // }
#endif
    }
    // Send messages on outportals
    sendLinesOut();
    sendCornersOut();
    ballOn = ballDetected;
    updateVisionBall();
    updateObstacleBox();
    sendCenterCircle();
}



void VisionModule::sendLinesOut()
{
    // Mark repeat lines (already found in bottom camera) in top camera
    for (int i = 0; i < fieldLines[0]->size(); i++) {
        for (int j = 0; j < fieldLines[1]->size(); j++) {
            FieldLine& topField = (*(fieldLines[0]))[i];
            FieldLine& botField = (*(fieldLines[1]))[j];
            for (int k = 0; k < 2; k++) {
                const GeoLine& topGeo = topField[k].field();
                const GeoLine& botGeo = botField[k].field();
                if (topGeo.error(botGeo) < 0.3) // TODO constant
                    (*(fieldLines[0]))[i].repeat(true);
            }
        }
    }

    // Outportal results
    // NOTE repeats are not outportaled
    messages::FieldLines pLines;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < fieldLines[i]->size(); j++) {
            messages::FieldLine* pLine = pLines.add_line();
            FieldLine& line = (*(fieldLines[i]))[j];
            if (line.repeat()) continue;

            for (int k = 0; k < 2; k++) {
                messages::HoughLine pHough;
                HoughLine& hough = line[k];

                pHough.set_r(hough.field().r());
                pHough.set_t(hough.field().t());
                pHough.set_ep0(hough.field().ep0());
                pHough.set_ep1(hough.field().ep1());

                if (hough.field().r() < 0)
                    pLine->mutable_outer()->CopyFrom(pHough);
                else
                    pLine->mutable_inner()->CopyFrom(pHough);
            }

            pLine->set_id(static_cast<int>(line.id()));
            pLine->set_index(static_cast<int>(line.index()));
        }
    }

    portals::Message<messages::FieldLines> linesOutMessage(&pLines);
    linesOut.setMessage(linesOutMessage);
}

// TODO repeats
void VisionModule::sendCornersOut()
{
    messages::Corners pCorners;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < cornerDetector[i]->size(); j++) {
            messages::Corner* pCorner = pCorners.add_corner();
            Corner& corner = (*(cornerDetector[i]))[j];

            // Rotate to post vision relative robot coordinate system
            double rotatedX, rotatedY;
            man::vision::translateRotate(corner.x, corner.y, 0, 0, -(M_PI / 2), rotatedX, rotatedY);

            pCorner->set_x(rotatedX);
            pCorner->set_y(rotatedY);
            pCorner->set_id(static_cast<int>(corner.id));
            pCorner->set_line1(static_cast<int>(corner.first->index()));
            pCorner->set_line2(static_cast<int>(corner.second->index()));
        }
    }

    portals::Message<messages::Corners> cornersOutMessage(&pCorners);
    cornersOut.setMessage(cornersOutMessage);
}

void VisionModule::updateVisionBall()
{
    portals::Message<messages::VisionBall> ball_message(0);

    Ball topBall = ballDetector[0]->best();
    Ball botBall = ballDetector[1]->best();

    bool top = false;
    Ball best = botBall;

    if (ballOn) {
        ballOnCount++;
        ballOffCount = 0;
    }
    else {
        ballOnCount = 0;
        ballOffCount++;
    }

    if (topBall.confidence() > botBall.confidence()) {
        best = topBall;
        top = true;
    }

    ball_message.get()->set_on(ballOn);
    ball_message.get()->set_frames_on(ballOnCount);
    ball_message.get()->set_frames_off(ballOffCount);
    ball_message.get()->set_intopcam(top);

    if (ballOn)
    {
        ball_message.get()->set_distance(best.dist);

        ball_message.get()->set_radius(best.blob.firstPrincipalLength());
        double bearing = atan(best.x_rel / best.y_rel);
        ball_message.get()->set_bearing(bearing);
        ball_message.get()->set_bearing_deg(bearing * TO_DEG);

        double angle_x = (best.imgWidth/2 - best.getBlob().centerX()) /
            (best.imgWidth) * HORIZ_FOV_DEG;
        double angle_y = (best.imgHeight/2 - best.getBlob().centerY()) /
            (best.imgHeight) * VERT_FOV_DEG;
        ball_message.get()->set_angle_x_deg(angle_x);
        ball_message.get()->set_angle_y_deg(angle_y);

        ball_message.get()->set_confidence(best.confidence());
        ball_message.get()->set_x(static_cast<int>(best.blob.centerX()));
        ball_message.get()->set_y(static_cast<int>(best.blob.centerY()));
    }

    ballOut.setMessage(ball_message);
}

void VisionModule::sendCenterCircle()
{ 
    portals::Message<messages::CenterCircle> ccm(0);

    ccm.get()->set_on(centerCircleDetected);
    ccm.get()->set_x(centerCircleDetector[0]->x());
    ccm.get()->set_y(centerCircleDetector[0]->y());
    
    centCircOut.setMessage(ccm);
}

void VisionModule::setColorParams(Colors* colors, bool topCamera)
{ 
    delete colorParams[!topCamera];
    colorParams[!topCamera] = colors;
}

const std::string VisionModule::getStringFromTxtFile(std::string path) 
{
    std::ifstream textFile;
    textFile.open(path);

    // Get size of file
    textFile.seekg (0, textFile.end);
    long size = textFile.tellg();
    textFile.seekg(0);

    // Read file into buffer and convert to string
    char* buff = new char[size];
    textFile.read(buff, size);
    std::string sexpText(buff);

    textFile.close();
    return (const std::string)sexpText;
}

#ifdef OFFLINE
	void VisionModule::setDebugDrawingParameters(nblog::SExpr* params) {
		std::cout << "In debug drawing parameters" << params->print() << std::endl;
		int cameraHorizon = params->get(1)->find("CameraHorizon")->get(1)->valueAsInt();
		int fieldHorizon = params->get(1)->find("FieldHorizon")->get(1)->valueAsInt();
		int debugHorizon = params->get(1)->find("DebugHorizon")->get(1)->valueAsInt();
		int debugField = params->get(1)->find("DebugField")->get(1)->valueAsInt();
		field[0]->setDrawCameraHorizon(cameraHorizon);
		field[0]->setDrawFieldHorizon(fieldHorizon);
		field[0]->setDebugHorizon(debugHorizon);
		field[0]->setDebugFieldEdge(debugField);
	}
#endif

/*
 Lisp data in config/colorParams.txt stores 32 parameters. Read lisp and
  load the three compoenets of a Colors struct, white, green, and orange,
  from the 18 values for either the top or bottom image. 
*/
Colors* VisionModule::getColorsFromLisp(nblog::SExpr* colors, int camera) 
{
    Colors* ret = new man::vision::Colors;
    nblog::SExpr* params;

    if (camera == 0) {
        params = colors->get(1)->find("Top")->get(1);
    } else if (camera == 1) {
        params = colors->get(1)->find("Bottom")->get(1);
    } else {
        params = colors->get(1);
    }

    colors = params->get(0)->get(1);

    ret->white. load(std::stof(colors->get(0)->get(1)->serialize()),
                     std::stof(colors->get(1)->get(1)->serialize()),
                     std::stof(colors->get(2)->get(1)->serialize()),
                     std::stof(colors->get(3)->get(1)->serialize()),
                     std::stof(colors->get(4)->get(1)->serialize()),
                     std::stof(colors->get(5)->get(1)->serialize()));

    colors = params->get(1)->get(1);

    ret->green. load(std::stof(colors->get(0)->get(1)->serialize()),
                     std::stof(colors->get(1)->get(1)->serialize()),
                     std::stof(colors->get(2)->get(1)->serialize()),
                     std::stof(colors->get(3)->get(1)->serialize()),
                     std::stof(colors->get(4)->get(1)->serialize()),
                     std::stof(colors->get(5)->get(1)->serialize()));

    colors = params->get(2)->get(1);

    ret->orange.load(std::stof(colors->get(0)->get(1)->serialize()),
                     std::stof(colors->get(1)->get(1)->serialize()),
                     std::stof(colors->get(2)->get(1)->serialize()),
                     std::stof(colors->get(3)->get(1)->serialize()),
                     std::stof(colors->get(4)->get(1)->serialize()),
                     std::stof(colors->get(5)->get(1)->serialize()));

    return ret;
}

void VisionModule::updateObstacleBox()
{
    int ht = 120;
    int wd = 160;
    int whiteBools[ht*wd];
    int edgeBools[wd*ht];
    // only want bottom camera

    robotImageObstacle->updateVisionObstacle(frontEnd[1]->whiteImage(),
                                             *(edges[1]), obstacleBox, whiteBools, edgeBools);

    // std::cout<<"about to set message for obstacle vision"<<std::endl;
    portals::Message<messages::RobotObstacle> boxOut(0);
    boxOut.get()->set_box_top(obstacleBox[0]);
    boxOut.get()->set_box_bottom(obstacleBox[1]);
    boxOut.get()->set_box_left(obstacleBox[2]);
    boxOut.get()->set_box_right(obstacleBox[3]);
    robotObstacleOut.setMessage(boxOut);

    std::cout<<"Obstacle box 2:"<<obstacleBox[0]<<", "<<obstacleBox[1]<<", "
             <<obstacleBox[2]<<", "<<obstacleBox[3]<<std::endl;

#ifdef OFFLINE
    // for (int i = 0; i < w*h; i++) {
    //     if (whiteBools[i] == 1) {
    //         field->drawPoint(i%w, i/w, 2);
    //     }
    // }

    for (int i = 0; i < wd*ht; i++) {
        if (edgeBools[i] == 1) {
            field[1]->drawPoint(i%wd, i/wd, 3);
        }
    }

    // bottom lines
    field[1]->drawLine(obstacleBox[3]-1,obstacleBox[1],obstacleBox[2]+1,obstacleBox[1],4);
    field[1]->drawLine(obstacleBox[3]-1,obstacleBox[1],obstacleBox[3]-1,0,4);
    field[1]->drawLine(obstacleBox[2]+1,0,obstacleBox[2]+1,obstacleBox[1],4);
#endif
}

void VisionModule::setCalibrationParams(std::string robotName) 
{
    setCalibrationParams(0, robotName);
    setCalibrationParams(1, robotName);
}

void VisionModule::setCalibrationParams(int camera, std::string robotName) 
{
    if (std::string::npos != robotName.find(".local")) {
        robotName.resize(robotName.find("."));
        if (robotName == "she-hulk")
            robotName = "shehulk";
    }
    if (robotName == "") {
        return;
    }

    nblog::SExpr* robot = calibrationLisp->get(1)->find(robotName);

    if (robot != NULL) {
        std::string cam = camera == 0 ? "TOP" : "BOT";
        double roll =  robot->find(cam)->get(1)->valueAsDouble();
        double tilt = robot->find(cam)->get(2)->valueAsDouble();
        delete calibrationParams[camera];
        calibrationParams[camera] = new CalibrationParams(roll, tilt);

        std::cerr << "Found and set calibration params for " << robotName;
        std::cerr << "Roll: " << roll << " Tilt: " << tilt << std::endl;
    }
}

void VisionModule::setCalibrationParams(CalibrationParams* params, bool topCamera)
{
    delete calibrationParams[!topCamera];
    calibrationParams[!topCamera] = params;
}

#ifdef USE_LOGGING
void VisionModule::logImage(int i) 
{
    bool blackStar = false;

    if (getenv("LOG_THIS") != NULL) {
        if (strcmp(getenv("LOG_THIS"), std::string("top").c_str()) == 0) {
            if (i != 0)
                return;
            else {
                setenv("LOG_THIS", "false", 1);
                blackStar = true;
                std::cerr << "pCal logging top log\n";
            }
        } else if (strcmp(getenv("LOG_THIS"), std::string("bottom").c_str()) == 0) {   
            if (i != 1)
                return;
            else {
                setenv("LOG_THIS", "false", 1);
                blackStar = true;
                std::cerr << "pCal logging bot log\n";
            }
        } else 
            return;
    }

    if (control::flags[control::tripoint]) {
        ++image_index;
        
        messages::YUVImage image;
        std::string image_from;
        if (!i) {
            image = topIn.message();
            image_from = "camera_TOP";
        } else {
            image = bottomIn.message();
            image_from = "camera_BOT";
        }
        
        long im_size = (image.width() * image.height() * 1);
        int im_width = image.width() / 2;
        int im_height= image.height();
        
        messages::JointAngles ja_pb = jointsIn.message();
        messages::InertialState is_pb = inertsIn.message();
        
        std::string ja_buf;
        std::string is_buf;
        std::string im_buf((char *) image.pixelAddress(0, 0), im_size);
        ja_pb.SerializeToString(&ja_buf);
        is_pb.SerializeToString(&is_buf);
        
        im_buf.append(is_buf);
        im_buf.append(ja_buf);
        
        std::vector<nblog::SExpr> contents;
        
        nblog::SExpr imageinfo("YUVImage", image_from, clock(), image_index, im_size);
        imageinfo.append(nblog::SExpr("width", im_width)   );
        imageinfo.append(nblog::SExpr("height", im_height) );
        imageinfo.append(nblog::SExpr("encoding", "[Y8(U8/V8)]"));
        contents.push_back(imageinfo);
        
        nblog::SExpr inerts("InertialState", "tripoint", clock(), image_index, is_buf.length());
        inerts.append(nblog::SExpr("acc_x", is_pb.acc_x()));
        inerts.append(nblog::SExpr("acc_y", is_pb.acc_y()));
        inerts.append(nblog::SExpr("acc_z", is_pb.acc_z()));
        
        inerts.append(nblog::SExpr("gyr_x", is_pb.gyr_x()));
        inerts.append(nblog::SExpr("gyr_y", is_pb.gyr_y()));
        
        inerts.append(nblog::SExpr("angle_x", is_pb.angle_x()));
        inerts.append(nblog::SExpr("angle_y", is_pb.angle_y()));
        contents.push_back(inerts);
        
        nblog::SExpr joints("JointAngles", "tripoint", clock(), image_index, ja_buf.length());
        joints.append(nblog::SExpr("head_yaw", ja_pb.head_yaw()));
        joints.append(nblog::SExpr("head_pitch", ja_pb.head_pitch()));

        joints.append(nblog::SExpr("l_shoulder_pitch", ja_pb.l_shoulder_pitch()));
        joints.append(nblog::SExpr("l_shoulder_roll", ja_pb.l_shoulder_roll()));
        joints.append(nblog::SExpr("l_elbow_yaw", ja_pb.l_elbow_yaw()));
        joints.append(nblog::SExpr("l_elbow_roll", ja_pb.l_elbow_roll()));
        joints.append(nblog::SExpr("l_wrist_yaw", ja_pb.l_wrist_yaw()));
        joints.append(nblog::SExpr("l_hand", ja_pb.l_hand()));

        joints.append(nblog::SExpr("r_shoulder_pitch", ja_pb.r_shoulder_pitch()));
        joints.append(nblog::SExpr("r_shoulder_roll", ja_pb.r_shoulder_roll()));
        joints.append(nblog::SExpr("r_elbow_yaw", ja_pb.r_elbow_yaw()));
        joints.append(nblog::SExpr("r_elbow_roll", ja_pb.r_elbow_roll()));
        joints.append(nblog::SExpr("r_wrist_yaw", ja_pb.r_wrist_yaw()));
        joints.append(nblog::SExpr("r_hand", ja_pb.r_hand()));

        joints.append(nblog::SExpr("l_hip_yaw_pitch", ja_pb.l_hip_yaw_pitch()));
        joints.append(nblog::SExpr("r_hip_yaw_pitch", ja_pb.r_hip_yaw_pitch()));

        joints.append(nblog::SExpr("l_hip_roll", ja_pb.l_hip_roll()));
        joints.append(nblog::SExpr("l_hip_pitch", ja_pb.l_hip_pitch()));
        joints.append(nblog::SExpr("l_knee_pitch", ja_pb.l_knee_pitch()));
        joints.append(nblog::SExpr("l_ankle_pitch", ja_pb.l_ankle_pitch()));
        joints.append(nblog::SExpr("l_ankle_roll", ja_pb.l_ankle_roll()));

        joints.append(nblog::SExpr("r_hip_roll", ja_pb.r_hip_roll() ));
        joints.append(nblog::SExpr("r_hip_pitch", ja_pb.r_hip_pitch() ));
        joints.append(nblog::SExpr("r_knee_pitch", ja_pb.r_knee_pitch() ));
        joints.append(nblog::SExpr("r_ankle_pitch", ja_pb.r_ankle_pitch() ));
        joints.append(nblog::SExpr("r_ankle_roll", ja_pb.r_ankle_roll() ));
        contents.push_back(joints);

        nblog::SExpr cal("CalibrationParams", "tripoint", clock(), image_index, 0);
        cal.append(nblog::SExpr(image_from, calibrationParams[i]->getRoll(), calibrationParams[i]->getTilt()));
        if (blackStar)
            cal.append(nblog::SExpr("BlackStar"));
        contents.push_back(cal);

        nblog::NBLog(NBL_IMAGE_BUFFER, "tripoint", contents, im_buf);
    }
}
#endif

}
}
