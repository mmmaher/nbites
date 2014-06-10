#include "TeammateInterpreterModule.h"

namespace man {
namespace mate {

TeammateInterpreterModule::TeammateInterpreterModule():
    portals::Module(),
    teammateInterpreterOutput(base())
{
    playerRole = 0;
    playerReliability = -1;

    zoneTracker.resize(NUM_PLAYERS_PER_TEAM);
    fallenBuffer.resize(FALLEN_BUFFER_SIZE);
    walkingBuffer.resize(WALKING_BUFFER_SIZE);
    shootingBuffer.resize(SHOOTING_BUFFER_SIZE);
    jumpingLocBuffer.resize(JUMPING_LOC_BUFFER_SIZE);

    previousX = previousY = 0;
    numFrames = 0;
}

TeammateInterpreterModule::~TeammateInterpreterModule()
{
}

void TeammateInterpreterModule:: run_()
{
    if (!(numFrames % TIME_BETWEEN_EXECUTIONS)) {
        worldModelIn.latch();
        interpretWorldModel(worldModelIn.message());
    }

}

void TeammateInterpreterModule::interpretWorldModel(messages::WorldModel newModel)
{
    // shift buffer to the right once: will drop oldest value
    // fills array[0] with a '0', which we want as default
    walkingBuffer = walkingBuffer.shift(-1);
    if (getSquaredDistance(newModel.my_x(), newModel.my_y(),
                           newModel.walking_to_x(), newModel.walking_to_y())
                           > (NOT_MOVING_RADIUS*NOT_MOVING_RADIUS)) {
        // I am moving somewhere
        walkingBuffer[0] = 1;
    }

    shootingBuffer = shootingBuffer.shift(-1);
    if (!(newModel.my_x() == newModel.kicking_to_x() &&
          newModel.my_y() == newModel.kicking_to_y())) {
        // am shooting when kicking_to() does not return current position
        shootingBuffer[0] = 1;
    }

    jumpingLocBuffer = jumpingLocBuffer.shift(-1);
    if (getSquaredDistance(newModel.my_x(), newModel.my_y(),
                           previousX, previousY)
                           < (TOO_BIG_LOC_JUMP * TOO_BIG_LOC_JUMP)) {
        jumpingLocBuffer[0] = 1;
    }
    previousX = newModel.my_x();
    previousY = newModel.my_y();

    // update playerRole
    interpretPlayerRole(newModel);
    interpretReliability(newModel.ball_age());


    portals::Message<messages::TeammateInterpreter> teammateInterpreterMessage(0);
    teammateInterpreterMessage.get()->set_reliability(playerReliability);
    teammateInterpreterMessage.get()->set_player_role(playerRole);
    teammateInterpreterOutput.setMessage(teammateInterpreterMessage);
}

void TeammateInterpreterModule::interpretReliability(float ballAge)
{
    float ballAgeValue, fallenValue;
    float walkingBadly = 0.f, shootingBadly = 0.f, jumpingLoc = 0.f;

    // first calulate normalized values
    ballAgeValue = ballAge * BALL_AGE_SCALER;
    fallenValue = fallenBuffer.sum() * FALLEN_SCALER;

    // now calculate binary values
    walkingBufferSum = walkingBuffer.sum();
    if ((float)walkingBufferSum / (float)WALKING_BUFFER_SIZE <
        PERC_SHOULD_WALK) {
        walkingBadly = 1.f * WALKING_SCALER;
    }
    shootingBufferSum = shootingBuffer.sum();
    if ((float)shootingBufferSum / (float)SHOOTING_BUFFER_SIZE <
        PERC_SHOULD_SHOOT) {
        shootingBadly = 1.f*SHOOTING_SCALER;
    }
    if ((float)jumpingLocBuffer.sum() / (float)JUMPING_LOC_BUFFER_SIZE >
        PERC_SHOULD_NOT_JUMP) {
        jumpingLoc = 1.f*JUMPING_LOC_SCALER;
    }

// SHOULD I RETURN PERCENTAGE OR JUST RANDOM NUMBER

    playerReliability = ballAgeValue + fallenValue + walkingBadly +
        shootingBadly + jumpingLoc;
}

/*
 * Initializes playerRole to where I am on the field.
 * If I am in a different zone than the one that corresponds to my
 *   current position, start to count the number of times I am in
 *   this new section. I will count 2x as much if I don't currently
 *   see the ball, as it makes my location more reliable.
 * If the counter in this section has passed the threshold, switch
 *   my role to this role.
 */
void TeammateInterpreterModule::interpretPlayerRole(messages::WorldModel newModel)
{
    int zone = getZone(newModel.my_x(),newModel.my_y());
    // if playerRole not defined, define to correspond to current zone
    if (!playerRole) {
        playerRole = zone;
        return;
    }
    // I am in my playerRole's zone, no need to update zone
    if (playerRole == zone) {
        return;
    }

    // IN NEW ZONE:
    if (!(zoneTracker[zone-1])) {
        // if not in this zone previously, clear all zones
        zoneTracker = 0;
    }
    // update counter in this zone
    if (newModel.ball_age() > BALL_AGE_THRESHOLD ||
        newModel.ball_age() == -1) {
        // can't see ball, weight this more
        zoneTracker[zone-1]+=2;
    } else {
        zoneTracker[zone-1]++;
    }
    // if counter in this zone is large enough, switch roles
    if (zoneTracker[zone-1] > SWITCH_ROLES_THRESHOLD) {
        playerRole = zone;
    }
}

int TeammateInterpreterModule::getZone(float x, float y)
{
    if ((x > FIELD_WHITE_LEFT_SIDELINE_X - GOAL_DEPTH) and
        (x <  FIELD_WHITE_LEFT_SIDELINE_X + GOALBOX_DEPTH) and
        (y > BLUE_GOALBOX_BOTTOM_Y) and
        (y < BLUE_GOALBOX_BOTTOM_Y + GOALBOX_WIDTH)){
        return GOALIE; // In goal box: Goalie zone
    } else if (x < DEFENDER_BOUNDARY && y >= MIDFIELD_Y) {
        return LEFT_DEFENDER; // L Defender zone
    } else if (x < DEFENDER_BOUNDARY && y < MIDFIELD_Y) {
        return RIGHT_DEFENDER; // R Defender zone
    } else if (x < CHASER_BOUNDARY) {
        return CHASER; // Chaser zone
    } else {
        return STRIKER; // Striker zone
    }
    return 0;
}

float TeammateInterpreterModule::getSquaredDistance (float x1, float y1, float x2, float y2)
{
    return ((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

} // namespace man
} // namespace context
