import pygame
from pygame.locals import *
from vector import Vector2
from constants import *
from algorithms import dijkstra, print_result, dijkstra_or_a_star
from random import randint, choice

class Entity(object):
    def __init__(self, node):
        self.name = None
        self.directions = {UP:Vector2(0, -1),DOWN:Vector2(0, 1), 
                          LEFT:Vector2(-1, 0), RIGHT:Vector2(1, 0), STOP:Vector2()}
        self.direction = STOP
        self.setSpeed(100)
        self.radius = 10
        self.collideRadius = 5
        self.color = WHITE
        self.node = node
        self.setPosition()
        self.target = node
        self.visible = True
        self.disablePortal = False
        self.goal = None
        self.directionMethod = self.randomDirection
        self.seek_or_flee = True    # seek=True, flee=False

        # FSM   
        # Added constants in "constants.py" file
        self.states = [SEEK, FLEE, WANDER,GO_CRAZY]
        self.myState = 0

    def setPosition(self):
        self.position = self.node.position.copy()
          
    def validDirection(self, direction):
        if direction is not STOP:
            if self.node.neighbors[direction] is not None:
                return True
        return False

    def getNewTarget(self, direction):
        if self.validDirection(direction):
            return self.node.neighbors[direction]
        return self.node

    def overshotTarget(self):
        if self.target is not None:
            vec1 = self.target.position - self.node.position
            vec2 = self.position - self.node.position
            node2Target = vec1.magnitudeSquared()
            node2Self = vec2.magnitudeSquared()
            return node2Self >= node2Target
        return False

    def reverseDirection(self):
        self.direction *= -1
        temp = self.node
        self.node = self.target
        self.target = temp
        
    def oppositeDirection(self, direction):
        if direction is not STOP:
            if direction == self.direction * -1:
                return True
        return False

    def goalDirection(self, directions):
        distances = []
        if self.seek_or_flee:
            for direction in directions:
                vec = self.node.position + self.directions[direction]*TILEWIDTH -  self.goal
                distances.append(vec.magnitudeSquared())
        else:
            for direction in directions:
                vec = self.goal - self.node.position + self.directions[direction]*TILEWIDTH
                distances.append(vec.magnitudeSquared())
        index = distances.index(min(distances))
        return directions[index]


    def setSpeed(self, speed):
        self.speed = speed * TILEWIDTH / 16

    def render(self, screen):
        if self.visible:
            p = self.position.asInt()
            pygame.draw.circle(screen, self.color, p, self.radius)

    def update(self, dt):
        # self.FSM_decision() ########## <========
        self.position += self.directions[self.direction]*self.speed*dt
         
        if self.overshotTarget():
            self.node = self.target
            directions = self.validDirections()
            direction = self.directionMethod(directions)
            self.target = self.getNewTarget(direction)
            if self.target is not self.node:
                self.direction = direction
            else:
                self.target = self.getNewTarget(self.direction)

            self.setPosition()
        
    #####

    #############
    # Executes Dijkstra from Ghost's previous node as start 
    # to pacman's target node as target.
    def getDijkstraPath(self, directions):
        lastPacmanNode = self.pacman.target
        lastPacmanNode = self.nodes.getPixelsFromNode(lastPacmanNode)
        ghostTarget = self.target
        ghostTarget = self.nodes.getPixelsFromNode(ghostTarget)

        # previous_nodes, shortest_path = dijkstra(self.nodes, ghostTarget)
        previous_nodes, shortest_path = dijkstra_or_a_star(self.nodes, ghostTarget, a_star=True)
        path = []
        node = lastPacmanNode
        while node != ghostTarget:
            path.append(node)
            node = previous_nodes[node]
        path.append(ghostTarget)
        path.reverse()
        # print(path)
        return path

    # Chooses direction in which to turn based on the dijkstra
    # returned path
    def goalDirectionDij(self, directions):
        path = self.getDijkstraPath(directions)
        print(path)
        ghostTarget = self.target
        ghostTarget = self.nodes.getPixelsFromNode(ghostTarget)
        path.append(ghostTarget)
        nextGhostNode = path[1]
        if ghostTarget[0] > nextGhostNode[0] and 2 in directions : #left
            return 2
        if ghostTarget[0] < nextGhostNode[0] and -2 in directions : #right
            return -2
        if ghostTarget[1] > nextGhostNode[1] and 1 in directions : #up
            return 1
        if ghostTarget[1] < nextGhostNode[1] and -1 in directions : #down
            return -1
        else:
            print(self.pacman.direction)
            print(directions)
            if -1 * self.pacman.direction in directions:
                return -1 * self.pacman.direction
            else: 
                return choice(directions)
        # up 1, down -1, left 2, right -2

    def validDirections(self):
        directions = []
        for key in [UP, DOWN, LEFT, RIGHT]:
            if self.validDirection(key):
                if key != self.direction * -1:
                    directions.append(key)
        if len(directions) == 0:
            directions.append(self.direction * -1)
        return directions

    def randomDirection(self, directions):
        return directions[randint(0, len(directions)-1)]

    def wanderRandom(self, directions):
        return self.randomDirection(directions)

    def wanderBiased(self, directions):
        previousDirection = self.direction
        if previousDirection in directions:
            nextDirProb = randint(1,100)
            if nextDirProb <= 50:
                return previousDirection
            else:
                directions.remove(previousDirection)
                return choice(directions)
        else:
            return self.wanderRandom(directions)
        
    def FSMstateChecker(self):
        if self.myState == SEEK: 
            self.directionMethod = self.goalDirectionDij
        elif self.myState == FLEE:
            self.seek_or_flee = False
            self.directionMethod = self.goalDirection
        elif self.myState == WANDER:
            self.directionMethod = self.wanderBiased
        else:
            self.myState = choice(self.states)