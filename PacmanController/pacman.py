import pygame
from pygame.locals import *
from vector import Vector2
from constants import *
from entity import Entity
from sprites import PacmanSprites
from random import choice
from algorithms import dijkstra, print_result, dijkstra_or_a_star

class Pacman(Entity):
    def __init__(self, node, nodes):
        Entity.__init__(self, node )
        self.name = PACMAN
        self.color = YELLOW
        self.direction = LEFT
        self.directionMethod = self.goalDirectionDij
        self.setBetweenNodes(LEFT)
        self.alive = True
        self.sprites = PacmanSprites(self)

        self.states = [SEEK_PELLET, RUN_AWAY, SEEK_GHOST]
        self.myState = SEEK_PELLET
        self.nodes=nodes
        self.pellets=[]
        self.currentPellet = None

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
        if self.currentPellet is not None:
            # print(f'update{self.currentPellet}')
            self.goal=self.currentPellet.position
        self.sprites.update(dt)
        Entity.update(self, dt)
        # self.position += self.directions[self.direction]*self.speed*dt
        # direction = self.getValidKey()
        if self.myState == SEEK_PELLET:
            self.seekPellet()

    def seekPellet(self):
        if self.currentPellet is not None:
            return
        else:
            self.seekNewPellet()

    def seekNewPellet(self):
        if len(self.pellets) > 0:
            next_node = self.node.neighbors[self.direction] if self.direction in self.node.neighbors else None
            if next_node and self.pelletExists(next_node.position):
                return
            nearestPellet = self.getNewNearestPellet()
            self.currentPellet = self.nodes.getNearestNode(nearestPellet.position)
        else:
            self.direction=STOP
    def pelletExists(self, position):
        for pellet in self.pellets:
            if pellet.position == position:
                return True
        return False
    def getNewNearestPellet(self):
        print(f'getNearestPellet {len(self.pellets)}')
        if len(self.pellets) > 0:
            nearest_pellet = None
            nearest_distance_squared = float('inf')
            pacman_position = self.node.position

            for pellet in self.pellets:
                pellet_position = pellet.position
                distance_squared = (pellet_position - pacman_position).magnitudeSquared()
                if distance_squared < nearest_distance_squared:
                    nearest_distance_squared = distance_squared
                    nearest_pellet = pellet

            return nearest_pellet
        else:
            return None

    def getValidKey(self):
        key_pressed = pygame.key.get_pressed()
        if key_pressed[K_UP]:
            return UP
        if key_pressed[K_DOWN]:
            return DOWN
        if key_pressed[K_LEFT]:
            return LEFT
        if key_pressed[K_RIGHT]:
            return RIGHT
        return STOP

    def eatPellets(self, pelletList):
        self.pellets = pelletList
        for pellet in pelletList:
            if self.collideCheck(pellet):
                self.currentPellet = None
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

    #############
    # Executes Dijkstra from Ghost's previous node as start
    # to pacman's target node as target.
    def getDijkstraPath(self, directions):
        lastPacmanNode = self.currentPellet
        lastPacmanNode = self.nodes.getVectorFromLUTNode(lastPacmanNode)
        ghostTarget = self.target
        ghostTarget = self.nodes.getVectorFromLUTNode(ghostTarget)

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
            # print(self.pacman.direction)
            # print(directions)
            if -1 * self.direction in directions:
                print('reverse')
                return -1 * self.direction
            else:
                return choice(directions)

        # up 1, down -1, left 2, right -2


    # Ghost can get stuck when having to reverse its direction
    def validDirections(self):
        directions = []
        for key in [UP, DOWN, LEFT, RIGHT]:
            if self.validDirection(key):
                # if key != self.direction * -1:
                directions.append(key)
        if len(directions) == 0:
            directions.append(self.direction * -1)
        return directions
