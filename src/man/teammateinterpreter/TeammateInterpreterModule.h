#pragma once

#include "RoboGrams.h"
#include "Common.h"

#include "FieldConstants.h"

#include <iostream>
#include <valarray>

#include "WorldModel.pb.h"

/**
 *
 * @Class to control estimate of other player roles and their reliability
 * @(how good of a player they are) in drop-in challenge games.
 *
 * @Uses the idea of "zones" of the field, defined as follows:
 *
 *   _____________
 *  |   |_____|   |       G = Goalie          1
 *  |             |       LD = Left Defender  2
 *  |      S      |       RD = Right Defender 3
 *  |_____________|       C = Chaser/Middie   4
 *  |_ _ _ C _ _ _|       S = Striker         5
 *  |______|______|
 *  |      |      |       Depending on what zones I am in most,
 *  |  LD  |  RD  |       we can infer what role I am playing
 *  |    __|__    |
 *  |___|__G__|___|+
 *               + \ (0,0)
 *
 * @For determining reliability: uses a scaling as follows:
 *
 *  ---------------------------------------------------------------------------
 * |as if|   VALUES NORMALIZED   |         BINARY VALUES (T/F)           |     |
 * |  in |---------------------------------------------------------------|     |
 * | sec.| ballAge | timesFallen | walkingBad | shootingBad | jumpingLoc |     |
 * |-----|---------------------------------------------------------------|-----|
 * | 600-|-10 mins-|             |            |             |            |-600 |
 * | 540-|    ^    |             |            |             |            |-540 |
 * | 480-|    |    |-all /3 mins-|            |             |--TRUE (1)--|-480 |
 * | 420-|    |    |     ^       |--TRUE (1)--|             |            |-420 |
 * | 360-|    |    |     |       |            |---TRUE (1)--|            |-360 |
 * | 300-|    |    |     |       |            |             |            |-300 |
 * | 240-|    |    |     |       |            |             |            |-240 |
 * | 180-|    |    |     v       |--FALSE (0)-|--FALSE (0)--|            |-180 |
 * | 120-|    |    |--0x /3 mins-|            |             |--FALSE (0)-|-120 |
 * |  60-|    v    |             |            |             |            |-60  |
 * |   0-|--0 mins-|             |            |             |            |-0   |
 *  ---------------------------------------------------------------------------
 *
 */

namespace man {
namespace mate {

const int GOALIE = 1;
const int LEFT_DEFENDER = 2;
const int RIGHT_DEFENDER = 3;
const int CHASER = 4;
const int STRIKER = 5;

const int TIME_BETWEEN_EXECUTIONS = 30; //only want to execute every second

const int FALLEN_BUFFER_SIZE = 180; // = 180 seconds / 3 minutes
const int WALKING_BUFFER_SIZE = 120; // = 120 seconds / 2 minutes
const int SHOOTING_BUFFER_SIZE = 180; // = 180 seconds / 3 minutes
const int JUMPING_LOC_BUFFER_SIZE = 120; // = 120 seconds / 2 minutes

const float BALL_AGE_SCALER = 1.f / 30.f; // converts from FPS to seconds
const float FALLEN_SCALER = 1.f;
const float WALKING_SCALER = 240.f;
const float SHOOTING_SCALER = 180.f;
const float JUMPING_LOC_SCALER = 360.f;

const float BALL_AGE_START = 0.f;
const float FALLEN_START = 120.f;
const float WALKING_START = 180.f;
const float SHOOTING_START = 180.f;
const float JUMPING_LOC_START = 120.f;

const float PERC_SHOULD_WALK = .3f;
const float PERC_SHOULD_SHOOT = .1f;
const float PERC_SHOULD_NOT_JUMP = .3f;

const int NOT_MOVING_RADIUS = 5;
const int TOO_BIG_LOC_JUMP = 10;

const float DEFENDER_BOUNDARY = CENTER_FIELD_X - CENTER_FIELD_X / 4.f;
const float CHASER_BOUNDARY = CENTER_FIELD_X + 2.f*(CENTER_FIELD_X / 5.f);

const int BALL_AGE_THRESHOLD = 30;
const int SWITCH_ROLES_THRESHOLD = 5;

class TeammateInterpreterModule : public portals::Module
{
public:
    TeammateInterpreterModule();
    virtual ~TeammateInterpreterModule();

    virtual void run_();

    portals::InPortal<messages::WorldModel> worldModelIn;
    portals::OutPortal<messages::TeammateInterpreter> teammateInterpreterOutput;

private:
    void interpretWorldModel(messages::WorldModel newModel);
    void interpretReliability(float ballAge);
    void interpretPlayerRole(messages::WorldModel newModel);
    int getZone(float x, float y);
    float getSquaredDistance(float x1, float y1, float x2, float y2);

    std::valarray<int> zoneTracker;
    std::valarray<int> fallenBuffer;
    std::valarray<int> walkingBuffer;
    std::valarray<int> shootingBuffer;
    std::valarray<int> jumpingLocBuffer;

    int numFrames; //num frames elapsed

    int shootingBufferSum;
    int walkingBufferSum;

    float previousX;
    float previousY;

    float playerReliability;
    float playerRole;
};

} // namespace context
} // namespace man
