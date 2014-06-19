import math
import noggin_constants as constants

CLAIM_EXPIRATION = 5
HEADING_WEIGHT = .5
CLAIM_DISTANCE = 50

def shouldCedeDropInClaim(player):
    if not player.useClaims:
        return False
    if player.name != 'pDropIn':
        return False

    playerWeight = weightedDistAndHeading(player.brain.ball.distance, \
                                              player.brain.loc.h, player.brain.ball.bearing_deg)
    playerIndex = 0
    for i in xrange (constants.NUM_PLAYERS_PER_TEAM):
        mate = player.brain.teamMembers[i]
        if (mate.playerNumber == player.brain.playerNumber):
            playerIndex = i
            continue
        if (player.claimTracker[i] == 0) or (player.claimTimer[i] == -1) or not mate.active:
            player.claimTracker[i] = 0
            player.claimTimer[i] = -1
            continue

        # Now we get into actual claims
        if ((player.brain.time - mate.claimTimer[i]) > CLAIM_EXPIRATION):
            print "claim expired"
            player.claimTracker[i] = 0
            player.claimTimer[i] = -1
            continue # That claim has expired (Comm is probably lagging)

        mateWeight = weightedDistAndHeading(mate.ballDist, mate.h, mate.ballBearing)
        # TODO: think more about comm lag/check comm lag
        if (mateWeight < playerWeight):
            if mate.ballDist < CLAIM_DISTANCE:
                player.claimTracker[i] = 1
                player.claimTimer[i] = player.brain.time
                return True

        if amShooting(mate):
            if mate.ballDist < CLAIM_DISTANCE:
                # mate has claimed ball
                player.claimTracker[i] = 1
                player.claimTimer[i] = player.brain.time
                return True

    player.claimTracker[playerIndex] = 1
    player.claimTimer[playerIndex] = player.brain.time

    return False

#TODO: make this make use of amount of orbit necessary
def weightedDistAndHeading(distance, heading, ballBearing):
    if heading > 180:
        heading -= 360
    if heading < -180:
        heading += 360

    ballHeading = heading + ballBearing
    if math.fabs(ballHeading) > 90:
        distance += distance * HEADING_WEIGHT * math.fabs(math.cos(math.radians(ballHeading)))
    return distance

def amShooting(mate):
    if ((mate.x == mate.kickingToX) and
        (mate.y == mate.kickingToY)):
        return True
    return False
