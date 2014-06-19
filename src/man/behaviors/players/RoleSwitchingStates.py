"""
An FSA for monitoring teammates and switching between roles on the field
implemented as a hierarchical FSA.

The second to top level in player FSA.
"""

import RoleSwitchingTransitions as transitions
import BoxPositionConstants as BPConstants
import noggin_constants as NConstants
from ..util import *

ACCEPTED_RELIABILITY = .7
SWITCH_ROLES_NUMBER = 5

@defaultState('gameControllerResponder')
@ifSwitchNow(transitions.chaserIsOut, 'switchRoles')
@ifSwitchNow(transitions.dropInRoleOpen, 'switchDropInRole')
@superState('fallController')
def roleSwitcher(player):
    """
    Superstate for checking if we need to switch roles.
    """
    pass

def switchRoles(player):
    """
    State to decide who on the team should become the new chaser and switch accordingly.
    """
    BPConstants.setRoleConstants(player, player.openChaser)

    return player.goLater(player.gameState)

    # # Should I become the chaser?
    # # If we uncomment this: must make it useful for drop-in too
    # #     because player.brain.teamMembers have no info
    # for mate in player.brain.teamMembers:
    #     if (mate.active and mate.role != 1 and
    #        mate.role < player.role):
    #         # No, another player will do it, continue playing...
    #         print "We're not gonna switch!!!!"
    #         return player.goLater(player.gameState)

    # # Yes, become the chaser...
    # BPConstants.setRoleConstants(player, player.openChaser)

    # print "We are the chaser!"
    # # And continue playing...
    # return player.goLater(player.gameState)

def switchDropInRole(player):
    # If it's first frame as this role, clear all roleCounts
    if not player.roleTracker[player.openDropInRole-1]:
        player.roleTracker = [0] * NConstants.NUM_PLAYERS_PER_TEAM
    #update counter in this zone
    player.roleTracker[player.openDropInRole-1]+=1

#    print "RoleTracker: ",player.roleTracker[0],player.roleTracker[1],player.roleTracker[2],player.roleTracker[3],player.roleTracker[4]

    # if counter in this zone is large enough, switch roles
    if player.roleTracker[player.openDropInRole-1] > SWITCH_ROLES_NUMBER:
        BPConstants.setDropInRoleConstants(player, player.openDropInRole)

    return player.goLater(player.gameState)
