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
        self.ghost = None

    # def createNodesFromPellets(self, pelletList):
    #     print('createNodesFromPellets', pelletList)
    #     for pellet in pelletList:
    #         print(pellet.position.x, pellet.position.y)
            # self.nodes.addSingleNode(pellet.position.x, pellet.position.y)
    def getGhosts(self, ghosts):
        # print('getGhosts', ghosts)
        self.ghosts = ghosts

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
        print('---------------------------------')
            # print(f'update{self.currentPellet}')
        self.sprites.update(dt)
        self.updatePosition(dt)
        self.updateState()
        if self.myState == SEEK_PELLET:
            # print('seek pellet')
            # if self.currentPellet is not None:
                # self.goal = self.currentPellet.position
            # self.sprites.update(dt)
            self.seekPellet()
        if self.myState == RUN_AWAY:
            print('run away')
            self.runAway()
        if self.myState == SEEK_GHOST:
            print('seek ghost')
            # self.seekGhost()

    def updateState(self):
        # priority: 1 - seek ghost, 2 - run away, 3 - seek pellet 
        
        # check seek ghost
        if any(ghost.mode.current == 2 for ghost in self.ghosts) and False:
            self.myState = SEEK_GHOST
            # todo update
            self.directionMethod = self.goalDirectionDij

        else:
        # check nearest ghost position
            smallest_distance = float('inf')
            nearest_ghost = None
            for ghost in self.ghosts:
                # print('ghost position', ghost.position)
                # print('pacman position', self.position)
                distance = (ghost.position - self.position).magnitudeSquared()
                if distance < smallest_distance:
                    smallest_distance = distance
                    nearest_ghost = ghost
            # print('smallest_distance', smallest_distance)
            # print('nearest_ghost', nearest_ghost)
            if smallest_distance < 3500:
                self.myState = RUN_AWAY
                self.directionMethod = self.getRunAwayDirection
                self.ghost = nearest_ghost
            else:
                self.myState = SEEK_PELLET
                self.directionMethod = self.goalDirectionDij
        # 
    
    def updatePosition(self, dt):
        self.position += self.directions[self.direction]*self.speed*dt
        if self.overshotTarget():
            self.node = self.target
            directions = self.validDirections()
            direction = self.directionMethod(directions)
            print('before new tag', direction)
            self.target = self.getNewTarget(direction)
            print('self.target', self.target.position)
            if self.target is not self.node:
                self.direction = direction
            else:
                print('else ')
                self.target = self.getNewTarget(self.direction)
            self.setPosition()
    
    def seekPellet(self):
        if self.currentPellet is not None:
            self.goal=self.currentPellet.position
            return
        else:
            self.seekNewPellet()

    def runAway(self):
        self.goal = self.ghost.position
        # if self.overshotTarget():
        #         self.node = self.target
        #         directions = self.validDirections()
        #         direction = self.getRunAwayDirection(directions)
        #         print('before new tag',)
        #         self.target = self.getNewTarget(direction)
        #         if self.target is not self.node:
        #             self.direction = direction
        #         else:
        #             self.target = self.getNewTarget(self.direction)

        #         self.setPosition()
    
    def getRunAwayDirection(self, directions):
        distances = []
        print('getRunawayDirection', directions)
        for direction in directions:
            vec = self.node.position  + self.directions[direction]*TILEWIDTH - self.goal
            distances.append(vec.magnitudeSquared())
        print('distances', distances)
        index = distances.index(max(distances))
        print('index', index)
        return directions[index]
    
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
        node_positions = self.nodes.getListOfNodesVector()
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
            return nearest_pellet
        elif len(self.pellets) > 0:
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
                self.currentPellet = None
                return pellet
        return None
    
    def updatePellets(self, pelletList):
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
        path = self.getDijkstraPath(directions)
        # print('goalDirectionDij',path)
        # print('self.position', round(self.position.x), round(self.position.y))
        # print('self.target', self.target.position)
        # ghostTarget = self.target
        # ghostTarget = self.nodes.getVectorFromLUTNode(ghostTarget)
        nextNode = path[1]
        diff_x = abs(nextNode[0] - round(self.position.x))
        diff_y = abs(nextNode[1] - round(self.position.y))
        if diff_x > diff_y:
            # Movement in x direction
            if nextNode[0] < round(self.position.x) and 2 in directions: #left
                # print("Next node is to the left.")
                return 2
            elif nextNode[0] > round(self.position.x) and -2 in directions: #right
                # print("Next node is to the right.")
                return -2
            else:
                # print("Next node is horizontally aligned with the current node.")
                return 0
        else:
            # Movement in y direction
            if nextNode[1] < round(self.position.y) and 1 in directions: #up
                # print("Next node is above.")
                return 1
            elif nextNode[1] > round(self.position.y) and -1 in directions:
                # print("Next node is below.")
                return -1
            else:
                # print("Next node is vertically aligned with the current node.")
                return 0

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
