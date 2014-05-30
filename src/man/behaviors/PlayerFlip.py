import noggin_constants as Constants

UNFLIPPED = -1
NEUTRAL = 0
FLIPPED = 1

GOALIE_FLIP_VISION_BOUNDARY = 250
DEFENDER_FLIP_VISION_BOUNDARY = 300
DEFENDER_FLIP_LOC_BOUNDARY = 250

# Grey area of two ball locations considered to be the same
AGREE_ON_BALL_RADIUS = 70

class PlayerFlip():

    # THIS IS A HACK!
    # ... but until we have a world contextor or some such, it's a necessary one.
    def __init__(self, brain):
        self.brain = brain
        self.goalieFlip = []
        self.defender1Flip = []
        self.defender2Flip = []

    def run(self):
        """
        Check where the goalie and two defenders see the ball and where we do.
        Record if we're generally correct, or if our flipped location
        is generally correct, or if neither one agrees with the goalie/defenders.
        NOTE: ignore whenever the ball is in the middle 2x2 meter box.
        """

        for mate in self.brain.teamMembers:
            if mate.active and self.ball.vis.on and mate.ballOn:
                # Break if defender is too close to midfield
                if ((mate.isDefaultDefender or mate.isDefaultMiddle)
                    and mate.x > DEFENDER_FLIP_LOC_BOUNDARY):
                    break;
                # Break if ball is in the middle 2x2 meter box
                if (Location(mate.x, mate.y).inCenterCenter() or
                    self.brain.ball.inCenterCenter()):
                    break;

                mate_ball_x = mate.x + (mate.ballDist * math.cos(mate.ballBearing))
                mate_ball_y = mate.y + (mate.ballDist * math.sin(mate.ballBearing))

                if mate.isDefaultDefender:
                    value = self.flipCheck(self, mate_ball_x, mate_ball_y,
                                           DEFENDER_FLIP_VISION_BOUNDARY)
                    self.defender1Flip = updateFlipArrays(self, value,
                                                          self.defender1Flip)
                elif mate.isDefaultMiddle:
                    value = self.flipCheck(self, mate_ball_x, mate_ball_y,
                                           DEFENDER_FLIP_VISION_BOUNDARY)
                    self.defender2Flip = updateFlipArrays(self, value,
                                                          self.defender2Flip)
                elif mate.isDefaultGoalie:
                    value = self.flipCheck(self, mate_ball_x, mate_ball_y,
                                           GOALIE_FLIP_VISION_BOUNDARY)
                    self.goalieFlip = updateFlipArrays(self, value,
                                                       self.goalieFlip)

        # If I've decided I should flip enough times, flip
        if ((len(self.defender1Flip) == 10 and sum(self.defender1Flip) > 6) or
            (len(self.defender2Flip) == 10 and sum(self.defender2Flip) > 6) or
            (len(self.goalieFlip) == 10 and sum(self.goalieFlip) > 6)):
            print("Defender flipping player")
            self.flipLoc()

            # Reset all lists so we don't flip again next frame
            self.defender1Flip = []
            self.defender2Flip = []
            self.goalieFlip=[]self.resetLists()


    def flipCheck(self, mate_ball_x, mate_ball_y, boundaryLine):
        if (mate_ball_x < boundaryLine and
            self.loc.x > Constants.MIDFIELD_X and
            self.ball.x > Constants.MIDFIELD_X):
            # I'm probably flipped
            return FLIPPED

        mate_ball_location = Location(mate_ball_x, mate_ball_y)
        my_flipped_ball = Location(Constants.FIELD_GREEN_WIDTH - self.ball.x,
                                   Constants.FIELD_GREEN_HEIGHT - self.ball.y)

        if self.brain.ball.distTo(mate_ball_location) < AGREE_ON_BALL_RADIUS:
            return UNFLIPPED
        elif my_flipped_ball.distTo(mate_ball_location) < AGREE_ON_BALL_RADIUS:
            return FLIPPED
        else:
            return NEUTRAL

    def updateFlipArrays(self, flipValue, flipList):
        flipArray.append(flipValue)
        if len(flipList) > 10:
            flipList.pop(0)
        return flipList

    def flipLoc(self):
        print ("My position was (" + str(self.loc.x) + ", " + str(self.loc.y) + ", " + str(self.loc.h) +
               ") and the ball's position was " + str(self.ball.x) + ", " + str(self.ball.y) + ")")

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
