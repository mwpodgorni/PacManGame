from constants import *
def seekPellet(self):
    self.activeGoal = getNewNearestPellet(self).position
    self.activeNodes = self.nodes
    self.directionMethod = self.goalDirectionDij

def runAway(self):
    distance, ghost = self.getNearestGhost([CHASE, SCATTER])
    print('getNearestGhost', ghost.position)
    self.activeGoal = ghost.position
    self.activeNodes = self.nodes
    self.directionMethod = self.getRunAwayDirection
    
def seekGhost(self):
    distance, ghost = self.getNearestGhost([FREIGHT])
    self.activeGoal = ghost.position
    self.activeNodes = self.nodes
    self.directionMethod = self.goalDirectionDij


def getNewNearestPellet(self):
    nearest_pellet = None
    nearest_distance_squared = float('inf')
    pacman_position = self.position
    for pellet in self.leftoverPellets:
        pellet_position = pellet.position
        distance_squared = (pellet_position - pacman_position).magnitudeSquared()
        if distance_squared < nearest_distance_squared:
            nearest_distance_squared = distance_squared
            nearest_pellet = pellet
    return nearest_pellet

