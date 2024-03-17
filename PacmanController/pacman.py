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
        self.pelletNodes = []

    # def createNodesFromPellets(self, pelletList):
    #     print('createNodesFromPellets', pelletList)
    #     for pellet in pelletList:
    #         print(pellet.position.x, pellet.position.y)
            # self.nodes.addSingleNode(pellet.position.x, pellet.position.y)
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
        # print('---------------------------------')
        if self.currentPellet is not None:
            # print(f'update{self.currentPellet}')
            self.goal=self.currentPellet.position
        self.sprites.update(dt)
        Entity.update(self, dt)
        # self.position += self.directions[self.direction]*self.speed*dt
        # direction = self.getValidKey()
        if self.myState == SEEK_PELLET:
            self.seekPellet()
        if self.myState == RUN_AWAY:
            self.runAway()

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
        #  the problem is that the pellets dont match with the nodes 
        # we need to find new pellet that is on the node
        # print("HEH", self.nodes.getListOfNodesVector())
        node_positions = self.nodes.getListOfNodesVector()
        print("node_positions", node_positions) 
        # for pellet in self.pellets:
        #     print(pellet.position)
        valid_pellets = [pellet for pellet in self.pellets if (pellet.position.x, pellet.position.y) in node_positions]
        if len(valid_pellets) > 0:
            nearest_pellet = None
            nearest_distance_squared = float('inf')
            pacman_position = self.position

            for pellet in valid_pellets:
                pellet_position = pellet.position
                distance_squared = (pellet_position - pacman_position).magnitudeSquared()
                if distance_squared < nearest_distance_squared:
                    nearest_distance_squared = distance_squared
                    nearest_pellet = pellet

            print(f'getNearestPellet {nearest_pellet.position}')
            return nearest_pellet
        elif len(self.pellets) > 0:
            print("ELLLSE")
            # for pellet in self.pellets:
                # self.nodes.addSingleNode(pellet.position.x, pellet.position.y)
            # return self.getNewNearestPellet()
            nearest_pellet = None
            nearest_distance_squared = float('inf')
            pacman_position = self.position

            for pellet in self.pellets:
                pellet_position = pellet.position
                distance_squared = (pellet_position - pacman_position).magnitudeSquared()
                if distance_squared < nearest_distance_squared and self.nodes.getNearestNode(pellet.position) is not self.nodes.getNearestNode(self.position):
                    nearest_distance_squared = distance_squared
                    nearest_pellet = pellet
            return self.nodes.getNearestNode(nearest_pellet.position)
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
                print('eatpellet')
                self.currentPellet = None
                return pellet
        return None
    def updatePellets(self, pelletList):
        print('updatePellets')
        self.pellets = pelletList

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
        currentPelletNode = self.currentPellet
        currentPelletNode = self.nodes.getVectorFromLUTNode(currentPelletNode)
        pacmanPosition = self.nodes.getNearestNode(self.position)
        pacmanPosition = self.nodes.getVectorFromLUTNode(pacmanPosition)

        # previous_nodes, shortest_path = dijkstra(self.nodes, ghostTarget)
        previous_nodes, shortest_path = dijkstra_or_a_star(self.nodes, pacmanPosition, a_star=True)
        # print('previous_nodes',previous_nodes)
        # print('shortest_path',shortest_path)
        path = []
        node = currentPelletNode
        while node != pacmanPosition:
            path.append(node)
            node = previous_nodes[node]
        path.append(pacmanPosition)
        path.reverse()
        # print(path)
        return path

    # Chooses direction in which to turn based on the dijkstra
    # returned path
    def goalDirectionDij(self, directions):
        print('----------------------')
        path = self.getDijkstraPath(directions)
        print('goalDirectionDij',path)
        print('self.position', round(self.position.x), round(self.position.y))
        print('self.target', self.target.position)
        # ghostTarget = self.target
        # ghostTarget = self.nodes.getVectorFromLUTNode(ghostTarget)
        nextNode = path[1]
        diff_x = abs(nextNode[0] - round(self.position.x))
        diff_y = abs(nextNode[1] - round(self.position.y))
        if diff_x > diff_y:
            # Movement in x direction
            if nextNode[0] < round(self.position.x) and 2 in directions: #left
                print("Next node is to the left.")
                return 2
            elif nextNode[0] > round(self.position.x) and -2 in directions: #right
                print("Next node is to the right.")
                return -2
            else:
                print("Next node is horizontally aligned with the current node.")
                return 0
        else:
            # Movement in y direction
            if nextNode[1] < round(self.position.y) and 1 in directions: #up
                print("Next node is above.")
                return 1
            elif nextNode[1] > round(self.position.y) and -1 in directions:
                print("Next node is below.")
                return -1
            else:
                print("Next node is vertically aligned with the current node.")
                return 0



        # path.append(ghostTarget)
        # nextGhostNode = path[1]
        # print('ghostTarget[0]',ghostTarget[0])
        # print('nextGhostNode[0]',nextGhostNode[0])
        # if ghostTarget[0] >= nextGhostNode[0] and 2 in directions : #left
        #     print('if1')
        #     return 2
        # if ghostTarget[0] <= nextGhostNode[0] and -2 in directions : #right
        #     print('if2')
        #     return -2
        # if ghostTarget[1] >= nextGhostNode[1] and 1 in directions : #up
        #     print('if3')
        #     return 1
        # if ghostTarget[1] <= nextGhostNode[1] and -1 in directions : #down
        #     print('if4')
        #     return -1
        # else:
        #     print('else')
        #     # print(self.pacman.direction)
        #     # print(directions)
        #     if -1 * self.direction in directions:
        #         print('reverse')
        #         return -1 * self.direction
        #     else:
        #         return choice(directions)

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
