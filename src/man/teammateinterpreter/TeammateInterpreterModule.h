#pragma once

#include "RoboGrams.h"
#include "Common.h"

#include "FieldConstants.h"

#include <iostream>
#include <valarray>

#include "WorldModel.pb.h"

/**
 * @author: Megan Maher, June 2014
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
 *  --------------------------------------------------------------------
 * |   |         VALUES NORMALIZED        |    BINARY VALUE (T/F)   |   |
 * |   |------------------------------------------------------------|   |
 * |   | ballAge | timesFallen | shooting | walkingBad | jumpingLoc |   |
 * |---|------------------------------------------------------------|   |
 * |10-|--5 mins-|1/3 of 3 mins|          |            |--TRUE (1)--|-10|
 * | 9-|    ^    |     ^       |          |            |            |-9 |
 * | 8-|    |    |     |       |          |            |            |-8 |
 * | 7-|  (-1)   |     |       |          |--TRUE (1)--|            |-7 |
 * | 6-|    |    |     |       |0% of time|            |            |-6 |
 * | 5-|    |    |     |       |    ^     |            |            |-5 |
 * | 4-|    |    |     |       |    |     |            |            |-4 |
 * | 3-|    |    |     v       |    |     |--FALSE (0)-|            |-3 |
 * | 2-|    |    |-0x in 3 min-|    |     |            |--FALSE (0)-|-2 |
 * | 1-|    v    |             |    v     |            |            |-1 |
 * | 0-|--0 mins-|             |  >33.3%  |            |            |-0 |
 *  --------------------------------------------------------------------
 *
 * @ The current information for each of the columns above:
 *     Ball Age:     - Given to us in frames/second, can be up to 10 minutes,
 *                     given a -1 value if robot has not seen ball yet
 *                   - If value is > 5 minutes, ballAge value will be greater
 *                     than 10 without adjusting the max value used for scaling
 *     Times Fallen: - Calculated in buffer of 3 minutes-long
 *                   - If fallen more than 1/3 of time, then timesFallen value
 *                     will be > 10 without adjusting max value used for scaling
 *     Shooting:     - Calculated in buffer of 3 minutes long
 *                   - If shooting more than 1/3 of the time, value will be less
 *                     than 0, without adjusting min value used for scaling
 *     Walking:      - If walking at least 1/3 of the time, given value of 3,
 *                     otherwise it is not enough and given value of 7
 *     Jumping Loc:  - If jumping loc around >= 1/3 of the time, given value of
 *                     10 for too much, otherwise it is okay: value of 2
 *
 * @ This makes min value for scaling (0 + 2 + 0 + 3 + 2) = 7
 * @ and max value for scaling (10 + 10 + 6 + 7 + 10) = 43
 * @ Therefore: to create reliability between 0 and 1, subtract 7 and
 * @      multiply by 1 / range of scale (MAX - MIN)
 */

namespace man {
namespace mate {

const int GOALIE = 1;
const int LEFT_DEFENDER = 2;
const int RIGHT_DEFENDER = 3;
const int CHASER = 4;
const int STRIKER = 5;

const int FRAMES_BETWEEN_EXECUTIONS = 30; //only want to execute every second

const int FALLEN_BUFFER_SIZE = 180; // = 180 seconds = 3 minutes
const int SHOOTING_BUFFER_SIZE = 180; // = 180 seconds = 3 minutes
const int WALKING_BUFFER_SIZE = 120; // = 120 seconds = 2 minutes
const int JUMPING_LOC_BUFFER_SIZE = 120; // = 120 seconds = 2 minutes

const float BALL_AGE_SCALE = 2.f / (30.f * 30.f * 60.f); // from FPS to minutes*2
const float FALLEN_SCALE = 6.f / 60.f; // 6/1min = scale, /60 = "minutes"
const float SHOOTING_SCALE = -18.f; // 1/3t -> -6, t=-18 (will add 6 to answer)
const float WALKING_SCALE = 4.f; // range
const float JUMPING_LOC_SCALE = 8.f; // range

const float BALL_AGE_START = 0.f;
const float FALLEN_START = 2.f;
const float SHOOTING_START = 6.f;
const float WALKING_START = 3.f;
const float JUMPING_LOC_START = 2.f;

const float MIN_RELIABILITY = BALL_AGE_START + FALLEN_START + WALKING_START +
    SHOOTING_START + JUMPING_LOC_START; // 7
const float MAX_RELIABILITY = 43.f; // Change to constant
const float RELIABILITY_SCALE = 1.f / (MAX_RELIABILITY - MIN_RELIABILITY);

const float PERC_SHOULD_WALK = 1.f / 3.f;
const float PERC_SHOULD_SHOOT = 1.f / 3.f;
const float PERC_SHOULD_NOT_JUMP = 1.f / 3.f;

const float NOT_MOVING_RADIUS = 5.f;
const float TOO_BIG_LOC_JUMP = 10.f;

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
    void interpretReliability(int ballAge);
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
    int playerRole;
};

} // namespace context
} // namespace man
