import pygame
from pygame.locals import *
from vector import Vector2
from constants import *
from entity import Entity
from sprites import PacmanSprites
from algorithms import dijkstra, print_result, dijkstra_or_a_star
from random import choice
import random
class Pacman(Entity):
    def __init__(self, node):
        Entity.__init__(self, node )
        self.name = PACMAN    
        self.color = YELLOW
        self.direction = LEFT
        self.setBetweenNodes(LEFT)
        self.alive = True
        self.sprites = PacmanSprites(self)
        # added fields
        self.ghosts = None
        self.directionMethod = self.goalDirection
        self.leftoverPellets = []
        self.nodes=None
        self.myState=None


    def getGhosts(self, ghost):
        self.ghosts = ghost
    def getNodes(self, nodes):
        self.nodes=nodes
    def reset(self):
        Entity.reset(self)
        self.direction = LEFT
        self.setBetweenNodes(LEFT)
        self.alive = True
        self.image = self.sprites.getStartImage()
        self.sprites.reset()
    def die(self):
        self.alive = False
        self.direction = STOP

    def update(self, dt):	
        self.sprites.update(dt)
        self.position += self.directions[self.direction]*self.speed*dt
        self.updateState()


        # if self.myState == SEEK_PELLET:
            # print('state == seek pellet')
            # self.seekPellet()
        if self.myState == RUN_AWAY:
            self.goal = self.getNearestGhost([SCATTER, CHASE]).target.position
        if self.myState == SEEK_GHOST:
            self.goal = self.getNearestGhost([FREIGHT]).target

        if self.overshotTarget():
            self.node = self.target
            directions = self.getValidDirections()
            direction = self.directionMethod(directions)
            if self.node.neighbors[PORTAL] is not None:
                self.node = self.node.neighbors[PORTAL]
            self.target = self.getNewTarget(direction)
            if self.target is not self.node:
                self.direction = direction
            else:
                self.target = self.getNewTarget(self.direction)
            self.setPosition()

    def getValidDirections(self):
        directions = []
        if self.myState in [RUN_AWAY, SEEK_GHOST]:
            for key in [UP, DOWN, LEFT, RIGHT]:
                if self.validDirection(key):
                    directions.append(key)
        else:
            for key in [UP, DOWN, LEFT, RIGHT]:
                if self.validDirection(key):
                    if key != self.direction * -1:
                        directions.append(key)
        if len(directions) == 0:
            directions.append(self.direction * -1)
        return directions
    
    # SCATTER = 0
    # CHASE = 1
    # FREIGHT = 2
    # SPAWN = 3
    def updateState(self):
        nearest_ghost = self.getNearestGhost([SCATTER, CHASE])
        nearest_ghost2 = self.getNearestGhost([FREIGHT])

        distance = (nearest_ghost.position - self.position).magnitudeSquared() if nearest_ghost is not None else 5000
        distance2 = (nearest_ghost2.position - self.position).magnitudeSquared() if nearest_ghost2 is not None else 6000
        if distance < 4000:
            if self.myState!=RUN_AWAY:
                print('change to run away')
            self.myState = RUN_AWAY
            self.directionMethod = self.getRunAwayDirection
        elif distance2 < 5000:
            if self.myState!=SEEK_GHOST:
                print('change to seek ghosts')
            self.myState = SEEK_GHOST
            self.directionMethod = self.goalDirectionDij
        else:
            if self.myState!=SEEK_PELLET:
                print('change to seek pellet')
            self.myState = SEEK_PELLET
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
    
    def getRunAwayDirection(self, directions):
        distances = []
        if random.random() < 0.5:
            if self.direction in [LEFT, RIGHT]:
                if UP in directions:
                    if DOWN in directions:
                        print('choice1')
                        return choice([UP, DOWN])
                    else:
                        print('choice2')
                        return UP
                elif DOWN in directions:
                    print('choice3')
                    return DOWN
            elif self.direction in [UP, DOWN]:
                if LEFT in directions:
                    if RIGHT in directions:
                        print('choice4')
                        return choice([LEFT, RIGHT])
                    else:
                        print('choice5')
                        return LEFT
                elif RIGHT in directions:
                    print('choice6')
                    return RIGHT
        for direction in directions:
            vec = self.node.position  + self.directions[direction]*TILEWIDTH - self.goal
            distances.append(vec.magnitudeSquared())
        index = distances.index(max(distances))
        return directions[index]
    
    def goalDirection(self, directions):
        distances = []
        for direction in directions:
            vec = self.node.position  + self.directions[direction]*TILEWIDTH - self.goal
            distances.append(vec.magnitudeSquared())
        index = distances.index(max(distances))
        return directions[index]
    
    def getNearestGhost(self, states):
        smallest_distance = float('inf')
        nearest_ghost = None
        for ghost in self.ghosts:
            distance = (ghost.position - self.position).magnitudeSquared()
            if ghost.mode.current in states and distance < smallest_distance:
                smallest_distance = distance
                nearest_ghost = ghost
        return nearest_ghost 

    def eatPellets(self, pelletList):
        self.leftoverPellets = pelletList
        for pellet in pelletList:
            if self.collideCheck(pellet):
                return pellet
        return None    
    
    def collideGhost(self, ghost):
        return self.collideCheck(ghost)

    def collideCheck(self, other):
        d = self.position - other.position
        dSquared = d.magnitudeSquared()
        rSquared = (self.collideRadius + other.collideRadius)**2
        if dSquared <= rSquared:
            return True
        return False

    # Chooses direction in which to turn based on the dijkstra
    # returned path
    def goalDirectionDij(self, directions):
        path = self.getDijkstraPath(directions)

        ghostTarget = self.target
        ghostTarget = self.nodes.getVectorFromLUTNode(ghostTarget)
        path.append(ghostTarget)
        nextGhostNode = path[1]
        # print(directions)
        if ghostTarget[0] > nextGhostNode[0] and 2 in directions : #left
            return 2
        if ghostTarget[0] < nextGhostNode[0] and -2 in directions : #right
            return -2
        if ghostTarget[1] > nextGhostNode[1] and 1 in directions : #up
            return 1
        if ghostTarget[1] < nextGhostNode[1] and -1 in directions : #down
            return -1
        else:
            print('dijskstra else',directions)
            # if -1 * self.pacman.direction in directions:
            #     return -1 * self.pacman.direction
            # else:
            return choice(directions)
        # up 1, down -1, left 2, right -2


    #############
    # Executes Dijkstra from Ghost's previous node as start
    # to pacman's target node as target.
    def getDijkstraPath(self, directions):
        pellet = self.getNewNearestPellet() if self.myState == SEEK_PELLET else self.goal
        lastPacmanNode = (pellet.position.x, pellet.position.y)
        ghostTarget = self.target
        ghostTarget = self.nodes.getVectorFromLUTNode(ghostTarget)

        # previous_nodes, shortest_path = dijkstra(self.nodes, ghostTarget)
        previous_nodes, shortest_path = dijkstra_or_a_star(self.nodes, ghostTarget, a_star=True)
        path = []
        node = lastPacmanNode
        while node != ghostTarget:
            path.append(node)
            if node in previous_nodes:
                node = previous_nodes[node]
            else:
                break
        path.append(ghostTarget)
        path.reverse()
        # print(path)
        return path