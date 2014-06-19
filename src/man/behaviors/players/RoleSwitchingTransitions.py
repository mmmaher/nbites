import noggin_constants as NConstants

ACCEPTABLE_RELIABILITY = .4

def chaserIsOut(player):
    # First make sure I am not a drop in player
    if isDropInPlayer(player):
        return False
    """
    There is a chaser spot ready to be filled.
    """
    if not player.roleSwitching:
        return False

    if not player.gameState == "gamePlaying":
        return False

    if player.role == 4 or player.role == 5:
        return False

    for mate in player.brain.teamMembers:
        if (mate.role == 4 or mate.role == 5) and not mate.active:
            player.openChaser = mate.role
            if player.openChaser - 2  == player.role:
                return True

    return False

def dropInRoleOpen(player):
    if not isDropInPlayer(player):
        return False
    if not player.roleSwitching:
        return False
    if not player.gameState == "gamePlaying":
        return False

    roleGuesses = [0] * NConstants.NUM_PLAYERS_PER_TEAM

    for i in xrange(NConstants.NUM_PLAYERS_PER_TEAM):
        # if not myself and am reliable
        if (player.brain.playerRoleGuess[i] != 0 and
            player.brain.playerReliability[i] > ACCEPTABLE_RELIABILITY):
            roleGuesses[player.brain.playerRoleGuess[i] - 1]+=1

    goalieCount = roleGuesses[0]
    defenderCount = roleGuesses[1] + roleGuesses[2]
    chaserCount = roleGuesses[3] + roleGuesses[4]
    newRole = 0
    if chaserCount < 2: #at least one of the chasers is empty
        if roleGuesses[4] == 0:
            newRole = 5
        else: # roleGuesses[3] == 0
            newRole = 4
    elif defenderCount < 2:
        if roleGuesses[2] == 0:
            newRole = 3
        else:
            newRole = 2
    elif goalieCount == 0:
        print("Oh no! We don't think there's a goalie! We will be a striker.")
        newRole = 5
    else:
        print("Error: Our Player guesses don't add up!")
        newRole = 5

    # I am already playing the desired role
    if newRole == player.role:
        return False

#    print"switching role from ",player.role," to ",newRole
    player.openDropInRole = newRole
    return True

def isDropInPlayer(player):
    return (player.name == 'pDropIn')
