import time
import sys
import math

# Redirect standard error to standard out
_stderr = sys.stderr
sys.stderr = sys.stdout

# Packages and modules from super-directories
import noggin_constants as Constants
from objects import Location, RobotLocation

# Modules from this directory
from . import Leds
from . import robots
from . import GameController
from . import FallController
from . import PlayerFlip

# Packages and modules from sub-directories
from .headTracker import HeadTracker
from .typeDefs import TeamMember
from .navigator import Navigator
from .players import Switch
from .kickDecider import KickDecider

# Import message protocol buffers and interface
import interface
import LedCommand_proto
import GameState_proto
import WorldModel_proto
import RobotLocation_proto
import BallModel_proto
import PMotion_proto
import MotionStatus_proto
import VisionRobot_proto
import VisionField_proto
import ButtonState_proto
import FallStatus_proto
import StiffnessControl_proto

class Brain(object):
    """
    Class brings all of our components together and runs the behaviors
    """

    def __init__(self, teamNum, playerNum):
        """
        Class constructor
        """

        # Parse arguments
        self.playerNumber = playerNum
        self.teamNumber = teamNum

        self.counter = 0
        self.time = time.time()

        # Initalize the leds and game controller
        self.leds = Leds.Leds(self)

        # Initialize fallController
        self.fallController = FallController.FallController(self)

        # Retrieve our robot identification and set per-robot parameters
        self.CoA = robots.get_certificate()

        # coa is Certificate of Authenticity (to keep things short)
        print '\033[32m'+str(self.CoA)                              +'\033[0m'
        print '\033[32m'+"GC:  I am on team "+str(self.teamNumber)  +'\033[0m'
        print '\033[32m'+"GC:  I am player  "+str(self.playerNumber)+'\033[0m'

        # Information about the environment
        self.ball = None
        self.initTeamMembers()
        self.motion = None
        self.game = None
        self.locUncert = 0

        # FSAs
        self.player = Switch.selectedPlayer.SoccerPlayer(self)
        self.tracker = HeadTracker.HeadTracker(self)
        self.nav = Navigator.Navigator(self)

        # Not FSAs
        self.gameController = GameController.GameController(self)
        self.kickDecider = KickDecider.KickDecider(self)
        self.player.claimedBall = False
        self.play = Play.Play()
        self.playerFlip = PlayerFlip.PlayerFlip(self)

        # Message interface
        self.interface = interface.interface

    def initTeamMembers(self):
        self.teamMembers = []
        for i in xrange(Constants.NUM_PLAYERS_PER_TEAM):
            mate = TeamMember.TeamMember(self)
            mate.playerNumber = i + 1
            self.teamMembers.append(mate)

##
##--------------CONTROL METHODS---------------##
##
    def profile(self):
        if self.counter == 0:
            cProfile.runctx('self.run()',  self.__dict__, locals(),
                            'pythonStats')
            self.p = pstats.Stats('pythonStats')

        elif self.counter < 3000:
            self.p.add('pythonStats')
            cProfile.runctx('self.run()',  self.__dict__, locals(),
                            'pythonStats')

        elif self.counter == 3000:
            self.p.strip_dirs()
            self.p.sort_stats('cumulative')
            ## print 'PYTHON STATS:'
            ## self.p.print_stats()
            ## print 'OUTGOING CALLEES:'
            ## self.p.print_callees()
            ## print 'OUTGOING CALLEES:'
            ## self.p.print_callers()
            self.p.dump_stats('pythonStats')

        self.counter += 1

    def run(self):
        """
        Main control loop
        """
        # Update Environment
        self.time = time.time()

        # Update objects
        self.updateVisionObjects()
        self.updateMotion()
        self.updateLoc()
        self.getCommUpdate()

        # Behavior stuff
        # Order here is very important
        self.gameController.run()
        self.fallController.run()
        self.player.run()
        self.tracker.run()
        self.nav.run()

        # HACK for dangerous ball flipping loc
        self.playerFlip.run()

        # Set LED message
        self.leds.processLeds()

        # Set myWorldModel for Comm
        self.updateComm()

        # Flush the output
        sys.stdout.flush()

    def updateComm(self):
        output = self.interface.myWorldModel

        output.timestamp = int(self.time * 1000)

        output.my_x = self.loc.x
        output.my_y = self.loc.y
        output.my_h = self.loc.h

        output.my_uncert = self.locUncert

        output.ball_on = self.ball.vis.on

        output.ball_dist = self.ball.distance
        output.ball_bearing = self.ball.bearing_deg

        #TODO get actual uncertainties, or transition to rel_x
        output.ball_dist_uncert = 0
        output.ball_bearing_uncert = 0

        output.role = self.teamMembers[self.playerNumber-1].role

        output.active = self.teamMembers[self.playerNumber-1].active
        output.in_kicking_state = self.player.inKickingState

        output.claimed_ball = self.player.claimedBall

    def getCommUpdate(self):
        self.teamMembers[self.playerNumber - 1].updateMe()
        self.game = self.interface.gameState
        for i in range(len(self.teamMembers)):
            if (i == self.playerNumber - 1):
                continue
            self.teamMembers[i].update(self.interface.worldModelList()[i])

    def updateMotion(self):
        self.motion = self.interface.motionStatus

    def updateVisionObjects(self):
        """
        Update estimates of robot and ball positions on the field
        """
        self.ball = self.interface.filteredBall
        if (self.player.gameState == 'gameReady'
            or self.player.gameState == 'gameSet'):
            self.ball.x = Constants.CENTER_FIELD_X
            self.ball.y = Constants.CENTER_FIELD_Y

        self.yglp = self.interface.visionField.goal_post_l.visual_detection
        self.ygrp = self.interface.visionField.goal_post_r.visual_detection

    def activeTeamMates(self):
        activeMates = 0
        for i in xrange(Constants.NUM_PLAYERS_PER_TEAM):
            mate = self.teamMembers[i]
            if mate.active:
                activeMates += 1
        return activeMates

    def updateLoc(self):
        """
        Make Loc info a RobotLocation.
        """
        self.loc = RobotLocation(self.interface.loc.x,
                                 self.interface.loc.y,
                                 self.interface.loc.h * (180. / math.pi))
        self.locUncert = self.interface.loc.uncert

    def resetLocTo(self, x, y, h):
        """
        Sends a reset request to loc to reset to given x, y, h
        """
        self.interface.resetLocRequest.x = x
        self.interface.resetLocRequest.y = y
        self.interface.resetLocRequest.h = h * (math.pi / 180.)
        self.interface.resetLocRequest.timestamp = int(self.time * 1000)

    def resetInitialLocalization(self):
        """
        Reset loc according to team number and team color.
        Assumes manual positioning.
        Note: Loc uses truly global coordinates, and the
              blue goalbox constants always match up with our goal.
        """
        # Does this matter for the goalie? It really shouldn't...
        if self.playerNumber == 1:
            self.resetLocTo(Constants.MIDFIELD_X,
                            Constants.FIELD_WHITE_BOTTOM_SIDELINE_Y,
                            Constants.HEADING_UP)
        elif self.playerNumber == 2:
            self.resetLocTo(Constants.BLUE_GOALBOX_MIDPOINT_X,
                            Constants.FIELD_WHITE_TOP_SIDELINE_Y,
                            Constants.HEADING_DOWN)
        elif self.playerNumber == 3:
            self.resetLocTo(Constants.BLUE_GOALBOX_MIDPOINT_X,
                            Constants.FIELD_WHITE_BOTTOM_SIDELINE_Y,
                            Constants.HEADING_UP)
        elif self.playerNumber == 4:
            self.resetLocTo(Constants.BLUE_GOALBOX_CROSS_MIDPOINT_X,
                            Constants.FIELD_WHITE_TOP_SIDELINE_Y,
                            Constants.HEADING_DOWN)
        elif self.playerNumber == 5:
            self.resetLocTo(Constants.BLUE_GOALBOX_CROSS_MIDPOINT_X,
                            Constants.FIELD_WHITE_BOTTOM_SIDELINE_Y,
                            Constants.HEADING_UP)


    #@todo: HACK HACK HACK Mexico 2012 to make sure we still re-converge properly even if
    #we get manually positioned
    #should make this nicer (or at least the locations)
    # Broken as of 4/2013
    def resetSetLocalization(self):
        gameSetResetUncertainties = _localization.LocNormalParams(50, 200, 1.0)

        if self.gameController.teamColor == Constants.teamColor.TEAM_BLUE:
            if self.playerNumber == 1:
                self.resetLocTo(Constants.BLUE_GOALBOX_RIGHT_X,
                                Constants.FIELD_WHITE_BOTTOM_SIDELINE_Y,
                                Constants.HEADING_UP)
                if self.gameController.ownKickOff:
                    self.resetLocTo(Constants.LANDMARK_BLUE_GOAL_CROSS_X,
                                    Constants.CENTER_FIELD_Y,
                                    0,
                                    gameSetResetUncertainties)
                else:
                    self.resetLocTo(Constants.BLUE_GOALBOX_RIGHT_X,
                                    Constants.CENTER_FIELD_Y,
                                    0,
                                    gameSetResetUncertainties)
            # HACK: Figure out what this is supposed to do!
            #self.loc.resetLocToSide(True)
        else:
            if self.gameController.ownKickOff:
                self.resetLocTo(Constants.LANDMARK_YELLOW_GOAL_CROSS_X,
                                Constants.CENTER_FIELD_Y,
                                180,
                                gameSetResetUncertainties)
            else:
                self.resetLocTo(Constants.YELLOW_GOALBOX_LEFT_X,
                                Constants.CENTER_FIELD_Y,
                                180,
                                gameSetResetUncertainties)
            #self.loc.resetLocToSide(False)

    def checkSetLocalization(self):
        """
        Use during the first frame of the set state.
        If we think we are on the opponent's side of the field, either
            1) We didn't make it back and will be manually positioned or
            2) Our loc is wrong, and we could be anywhere.
        Reset to our own field cross. The loc system should be able to
        recover from there with high probability. 6/13/13
        """
        if self.loc.x > Constants.MIDFIELD_X:
            self.resetLocTo(Constants.LANDMARK_MY_FIELD_CROSS[0],
                            Constants.LANDMARK_MY_FIELD_CROSS[1],
                            Constants.HEADING_RIGHT)

    def resetLocalizationFromPenalty(self, top):
        """
        Resets localization from penalty.
        @param top: true is our goal is to our right, false if it is to our left.
        """
        if top:
            self.resetLocTo(Constants.LANDMARK_BLUE_GOAL_CROSS_X,
                            Constants.FIELD_WHITE_TOP_SIDELINE_Y,
                            Constants.HEADING_DOWN)
        else:
            self.resetLocTo(Constants.LANDMARK_BLUE_GOAL_CROSS_X,
                            Constants.FIELD_WHITE_BOTTOM_SIDELINE_Y,
                            Constants.HEADING_UP)

    def resetGoalieLocalization(self):
        """
        Resets the goalie's localization to the manual position in the goalbox.
        """
        self.resetLocTo(Constants.FIELD_WHITE_LEFT_SIDELINE_X,
                        Constants.MIDFIELD_Y,
                        Constants.HEADING_RIGHT)

    def resetPenaltyKickLocalization(self):
        self.resetLocTo(Constants.LANDMARK_OPP_FIELD_CROSS[0] - 1.0,
                        Constants.MIDFIELD_Y,
                        Constants.HEADING_RIGHT)
