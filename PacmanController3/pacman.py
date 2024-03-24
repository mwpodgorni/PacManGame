import pygame
from pygame.locals import *
from vector import Vector2
from constants import *
from entity import Entity
from sprites import PacmanSprites
from algorithms import dijkstra, print_result, dijkstra_or_a_star
from random import choice

class Pacman(Entity):
    def __init__(self, node):
        Entity.__init__(self, node )
        self.name = PACMAN    
        self.color = YELLOW
        self.direction = LEFT
        self.setBetweenNodes(LEFT)
        self.alive = True
        self.sprites = PacmanSprites(self)
        # ###########
        self.ghosts = None
        self.directionMethod = self.goalDirection
        self.leftoverPellets = []
        self.nodes=None


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


        if self.myState == SEEK_PELLET:
            print('state == seek pellet')
            # self.seekPellet()
        if self.myState == RUN_AWAY:
            print('state == run away')
            self.goal = self.getNearestGhost().position

        if self.myState == SEEK_GHOST:
            print('state == seek ghost')
            # self.seekGhost()
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
        if self.myState == RUN_AWAY:
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
    def updateState(self):
        # priority: 1 - seek ghost, 2 - run away, 3 - seek pellet

        # check seek ghost
        # if any(ghost.mode.current == 2 for ghost in self.ghosts):
        #     self.myState = SEEK_GHOST
        #     self.directionMethod = self.goalDirectionDij
        # else:
        # check nearest ghost position
        smallest_distance = float('inf')
        nearest_ghost = self.getNearestGhost()
        distance = (nearest_ghost.position - self.position).magnitudeSquared()

        # print('smallest_distance', smallest_distance)
        # print('nearest_ghost', nearest_ghost)
        if distance < 3000:
            self.myState = RUN_AWAY
            self.directionMethod = self.getRunAwayDirection
        else:
            self.myState = SEEK_PELLET
            self.directionMethod = self.goalDirectionDij
            # self.directionMethod = self.goalDirectionDij
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
        for direction in directions:
            vec = self.node.position  + self.directions[direction]*TILEWIDTH - self.goal
            distances.append(vec.magnitudeSquared())
        index = distances.index(max(distances))
        print('runaway direction', directions[index])
        return directions[index]
    def goalDirection(self, directions):
        distances = []
        for direction in directions:
            vec = self.node.position  + self.directions[direction]*TILEWIDTH - self.goal
            distances.append(vec.magnitudeSquared())
        index = distances.index(max(distances))
        return directions[index]
    
    def getNearestGhost(self):
        smallest_distance = float('inf')
        nearest_ghost = None
        for ghost in self.ghosts:
            distance = (ghost.position - self.position).magnitudeSquared()
            if distance < smallest_distance:
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
        if ghostTarget[0] > nextGhostNode[0] and 2 in directions : #left
            return 2
        if ghostTarget[0] < nextGhostNode[0] and -2 in directions : #right
            return -2
        if ghostTarget[1] > nextGhostNode[1] and 1 in directions : #up
            return 1
        if ghostTarget[1] < nextGhostNode[1] and -1 in directions : #down
            return -1
        else:
            print(directions)
            # if -1 * self.pacman.direction in directions:
            #     return -1 * self.pacman.direction
            # else:
            return choice(directions)
        # up 1, down -1, left 2, right -2


    #############
    # Executes Dijkstra from Ghost's previous node as start
    # to pacman's target node as target.
    def getDijkstraPath(self, directions):
        pellet = self.getNewNearestPellet()
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