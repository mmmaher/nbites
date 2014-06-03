import math

import noggin_constants as Constants
from objects import Location, RobotLocation

from .typeDefs import TeamMember

UNFLIPPED = -1
NEUTRAL = 0
FLIPPED = 1

GOALIE_FLIP_VISION_BOUNDARY = 250
GOALIE_MUTUAL_FLIP_BOUNDARY = 300
DEFENDER_FLIP_VISION_BOUNDARY = MIDFIELD_X - (CENTER_CIRCLE_RADIUS / 2) - 10
DEFENDER_FLIP_LOC_BOUNDARY = 250

# Grey area of two ball locations considered to be the same
AGREE_ON_BALL_RADIUS = 70

class PlayerFlip():

    # THIS IS A HACK!
    # ... but until we have a world contextor or some such, it's a necessary one.
    def __init__(self, brain):
        self.brain = brain
        self.goalieFlip = []
        self.defenderLFlip = []
        self.defenderRFlip = []
#        self.mutualChecker = []
#        self.mutualFlip = []

    def run(self):
        """
        Check where the goalie and two defenders see the ball and where we do.
        Record if we're generally correct, or if our flipped location
        is generally correct, or if neither one agrees with the goalie/defenders.
        NOTE: ignore whenever the ball is in the middle 2x2 meter box.
        """

        for mate in self.brain.teamMembers:
            if mate.active and self.brain.ball.vis.on and mate.ballOn:
                # Break if defender is too close to midfield
                if (mate.isDefaultDefender and mate.x > DEFENDER_FLIP_LOC_BOUNDARY):
                    break;
                # Need to convert to locations to check if in center
                mate_location = Location(mate.x, mate.y)
                my_location = Location(self.brain.ball.x, self.brain.ball.y)
                #print(str(my_location.x) + "," + str(my_location.y) + "; " + str(mate_location.x) + "," + str(mate_location.y) + "\n")
                # Break if ball is in the middle 2x2 meter box
                if (mate_location.inCenterCenter() or my_location.inCenterCenter()):
                    break;

                mate_ball_x = mate.x + (mate.ballDist * math.cos(mate.ballBearing))
                mate_ball_y = mate.y + (mate.ballDist * math.sin(mate.ballBearing))

                if mate.isDefaultLeftDefender():
                    value = self.flipCheck(mate_ball_x, mate_ball_y,
                                           DEFENDER_FLIP_VISION_BOUNDARY)
                    self.defenderLFlip = self.updateFlipArrays(value,
                                                          self.defenderLFlip)
#                    if value > 0: value = 2
#                    self.mutualChecker.append(value)
                elif mate.isDefaultRightDefender():
                    value = self.flipCheck(mate_ball_x, mate_ball_y,
                                           DEFENDER_FLIP_VISION_BOUNDARY)
                    self.defenderRFlip = self.updateFlipArrays(value,
                                                          self.defenderRFlip)
#                    if value > 0: value = 2
#                    self.mutualChecker.append(value)
                elif mate.isDefaultGoalie():
                    value = self.flipCheck(mate_ball_x, mate_ball_y,
                                           GOALIE_FLIP_VISION_BOUNDARY)
                    self.goalieFlip = self.updateFlipArrays(value,
                                                       self.goalieFlip)
#                    value = self.flipCheck(mate_ball_x, mate_ball_y,
#                                          GOALIE_MUTUAL_FLIP_BOUNDARY)
#                    if value > 0: value = 2
#                    self.mutualChecker.append(value)

#                # if 2/3 say flip, append 1 to mutual flip
#                if len(self.mutualChecker) == 3:
#                    if sum(self.mutualChecker) > 2:
#                        self.mutualFlip.append(1)
#                    else:
#                        self.mutualFlip.append(-1)
#                    self.mutualChecker = []

        # If I've decided I should flip enough times in any array, flip
#        if ((len(self.defenderLFlip) == 10 and sum(self.defenderLFlip) > 6) or
#            (len(self.defenderRFlip) == 10 and sum(self.defenderRFlip) > 6) or
#            (len(self.goalieFlip) == 10 and sum(self.goalieFlip) > 6)):

        if (len(self.defenderLFlip) == 10 and sum(self.defenderLFlip) > 6):
            print("Left Defender flipping player\n")
            self.flipLoc()

            # Reset all lists so we don't flip again next frame
            self.defenderLFlip = []
            self.defenderRFlip = []
            self.goalieFlip=[]
        elif (len(self.defenderRFlip) == 10 and sum(self.defenderRFlip) > 6):
            print ("Right Defender flipping player\n")
            self.flipLoc()

            # Reset all lists so we don't flip again next frame
            self.defender1Flip = []
            self.defender2Flip = []
            self.goalieFlip=[]

        elif (len(self.goalieFlip) == 10 and sum(self.goalieFlip) > 6):
            print ("Goalie flipping player\n")
            self.flipLoc()

            # Reset all lists so we don't flip again next frame
            self.defenderLFlip = []
            self.defenderRFlip = []
            self.goalieFlip=[]


    def flipCheck(self, mate_ball_x, mate_ball_y, boundaryLine):
        if (mate_ball_x < boundaryLine and
            self.brain.loc.x > Constants.MIDFIELD_X and
            self.brain.ball.x > Constants.MIDFIELD_X):
            # I'm probably flipped
            return FLIPPED

        # Convert to locations to use useful functions
        mate_ball_location = Location(mate_ball_x, mate_ball_y)
        my_location = Location(self.brain.ball.x, self.brain.ball.y)
        my_flipped_ball = Location(Constants.FIELD_GREEN_WIDTH - self.brain.ball.x,
                                   Constants.FIELD_GREEN_HEIGHT - self.brain.ball.y)

        if my_location.distTo(mate_ball_location) < AGREE_ON_BALL_RADIUS:
            return UNFLIPPED
        elif my_flipped_ball.distTo(mate_ball_location) < AGREE_ON_BALL_RADIUS:
            return FLIPPED
        else:
            return NEUTRAL

    def updateFlipArrays(self, flipValue, flipList):
        flipList.append(flipValue)
        if len(flipList) > 10:
            flipList.pop(0)
        return flipList

    def flipLoc(self):
        print ("My position was (" + str(self.brain.loc.x) + ", " + str(self.brain.loc.y) + ", " + str(self.brain.loc.h) +
               ") and the ball's position was " + str(self.brain.ball.x) + ", " + str(self.brain.ball.y) + ")")

        if (self.brain.playerNumber == TeamMember.DEFAULT_GOALIE_NUMBER):
            # Shouldn't happen: If I'm a goalie, reset to the penatly box.
            print "I am a goalie. Resetting loc to the goalbox."
            self.brain.resetGoalieLocalization()
            return

        reset_x = (-1*(self.brain.loc.x - Constants.MIDFIELD_X)) + Constants.MIDFIELD_X
        reset_y = (-1*(self.brain.loc.y - Constants.MIDFIELD_Y)) + Constants.MIDFIELD_Y
        reset_h = self.brain.loc.h + 180
        if reset_h > 180:
            reset_h -= 360
        self.brain.resetLocTo(reset_x, reset_y, reset_h)
