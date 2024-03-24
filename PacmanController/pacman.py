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
        print('goal',self.goal)
        print('node', self.node.position)
        print('target', self.target.position)
            # print(f'update{self.currentPellet}')
        self.sprites.update(dt)
        self.position += self.directions[self.direction]*self.speed*dt
        # self.updatePosition(dt)
        self.updateState()
        if self.myState == SEEK_PELLET:
            print('state == seek pellet')
            self.seekPellet()
        if self.myState == RUN_AWAY:
            print('state == run away')
            self.runAway()
        if self.myState == SEEK_GHOST:
            print('state == seek ghost')
            self.seekGhost()

    def updateState(self):
        # priority: 1 - seek ghost, 2 - run away, 3 - seek pellet

        # check seek ghost
        # if any(ghost.mode.current == 2 for ghost in self.ghosts):
        #     self.myState = SEEK_GHOST
        #     self.directionMethod = self.goalDirectionDij
        # else:
        # # check nearest ghost position
        #     smallest_distance = float('inf')
        #     nearest_ghost = None
        #     for ghost in self.ghosts:
        #         # print('ghost position', ghost.position)
        #         # print('pacman position', self.position)
        #         distance = (ghost.position - self.position).magnitudeSquared()
        #         if distance < smallest_distance:
        #             smallest_distance = distance
        #             nearest_ghost = ghost
        #     # print('smallest_distance', smallest_distance)
        #     # print('nearest_ghost', nearest_ghost)
        #     if smallest_distance < 3500:
        #         self.myState = RUN_AWAY
        #         self.directionMethod = self.getRunAwayDirection
        #         self.ghost = nearest_ghost
        #     else:
        self.myState = SEEK_PELLET
        self.directionMethod = self.goalDirectionDij
        #

    def updatePosition(self, dt):
        self.position += self.directions[self.direction]*self.speed*dt
        if self.overshotTarget():
            self.node = self.target
            directions = self.validDirections()
            # directions =  [d for d in directions if d != self.direction*-1]
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
        print('seekPellet')
        if self.overshotTarget():
            print('overshotTarget')
            self.node = self.target
            directions = self.validDirections()
            # directions =  [d for d in directions if d != self.direction*-1]
            direction = self.directionMethod(directions)
            print('directions', direction)
            self.target = self.getNewTarget(direction)
            print('self.target', self.target.position)
            if self.target is not self.node:
                self.direction = direction
            else:
                print('else ')
                self.target = self.getNewTarget(self.direction)
            self.setPosition()

        if self.currentPellet is not None:
            print('seek if')
            self.goal=self.currentPellet.position
            return
        else:
            print('seek else')
            self.seekNewPellet()

    def seekNewPellet(self):
        if len(self.pellets) > 0:
            # if next node in current direction exist and has pellet, just continue going
            # next_node = self.node.neighbors[self.direction] if self.direction in self.node.neighbors else None
            # next_node = next_node.neighbors[self.direction] if next_node and self.direction in next_node.neighbors else None
            # if next_node and self.pelletExists(next_node.position):
            #     print('seekNewPellet nextNode')
            #     self.currentPellet = next_node
            #     return
            print('seekNewPellet else')
            nearestPellet = self.getNewNearestPellet()
            self.currentPellet = self.nodes.getNearestNode(nearestPellet.position)
        else:
            self.direction=STOP

    def pelletExists(self, position):
        for pellet in self.pellets:
            if pellet.position == position:
                return True
        return False

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
    def seekGhost(self):
        smallest_distance = float('inf')
        nearest_ghost = None
        for ghost in self.ghosts:
            # print('ghost position', ghost.position)
            # print('pacman position', self.position)
            distance = (ghost.position - self.position).magnitudeSquared()
            if ghost.mode.current == 2  and distance < smallest_distance :
                smallest_distance = distance
                nearest_ghost = ghost
        self.goal = nearest_ghost.target.position
        print('seek ghost end', self.goal)
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

    def getNewNearestPellet(self):
        print('getNewNearestPellet')
        node_positions = self.nodes.getListOfNodesVector()
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
 

    def eatPellets(self, pelletList):
        self.pellets = pelletList
        for pellet in pelletList:
            if self.collideCheck(pellet):
                self.currentPellet = None
                self.seekPellet()
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
        # print('getDijkstraPath', self.goal)
        currentGoalNode = self.nodes.getNearestNode(self.goal)
        currentGoalNode = self.nodes.getVectorFromLUTNode(currentGoalNode)
        pacmanTarget = self.target
        pacmanTarget = self.nodes.getVectorFromLUTNode(pacmanTarget)

        # previous_nodes, shortest_path = dijkstra(self.nodes, ghostTarget)
        previous_nodes, shortest_path = dijkstra_or_a_star(self.nodes, pacmanTarget, a_star=True)
        # print('previous_nodes',previous_nodes)
        # print('currentGoalNode',currentGoalNode)
        # print('shortest_path',shortest_path)
        path = []
        node = currentGoalNode
        try:
            while node != pacmanTarget:
                path.append(node)
                # if node in previous_nodes:
                node = previous_nodes[node]
            path.append(pacmanTarget)
            path.reverse()
        except:
            print('error')
            print('current', node)
            for node in previous_nodes:
                print(node)
        # print(path)
        return path

    # Chooses direction in which to turn based on the dijkstra
    # returned path
    def goalDirectionDij(self, directions):
        path = self.getDijkstraPath(directions)


        pacmanTarget = self.target
        pacmanTarget = self.nodes.getVectorFromLUTNode(pacmanTarget)
        path.append(pacmanTarget)
        nextPacmanNode = path[1]
        if pacmanTarget[0] > nextPacmanNode[0] and 2 in directions : #left
            return 2
        if pacmanTarget[0] < nextPacmanNode[0] and -2 in directions : #right
            return -2
        if pacmanTarget[1] > nextPacmanNode[1] and 1 in directions : #up
            return 1
        if pacmanTarget[1] < nextPacmanNode[1] and -1 in directions : #down
            return -1
        else:
            print("GOAL DIRECTION ELSE")
            if -1 * self.direction in directions:
                return -1 * self.pacman.direction
            else:
                return choice(directions)
        # print('goalDirectionDij',path)
        # print('self.position', round(self.position.x), round(self.position.y))
        # print('self.target', self.target.position)
        # ghostTarget = self.target
        # ghostTarget = self.nodes.getVectorFromLUTNode(ghostTarget)
        # print('path', path)
        # print('directions', directions)
        # nextNode = path[1] if len(path)>1 else path[0]
        # diff_x = abs(nextNode[0] - round(self.position.x))
        # diff_y = abs(nextNode[1] - round(self.position.y))
        # if (diff_x > diff_y and (2 in directions or -2 in directions)) or (diff_x < diff_y and (1 not in directions and -1 not in directions)):
        #     # Movement in x direction
        #     if nextNode[0] < round(self.position.x) and 2 in directions: #left
        #         # print("Next node is to the left.")
        #         return 2
        #     elif nextNode[0] > round(self.position.x) and -2 in directions: #right
        #         # print("Next node is to the right.")
        #         return -2
        #     else:
        #         print("Next node is horizontally aligned with the current node.")
        #         if self.direction in directions:
        #             return self.direction
        #         elif len(directions):
        #             return choice(directions)
        #         return self.direction*-1
        # else:
        #     # Movement in y direction
        #     if nextNode[1] < round(self.position.y) and 1 in directions: #up
        #         # print("Next node is above.")
        #         return 1
        #     elif nextNode[1] > round(self.position.y) and -1 in directions:
        #         # print("Next node is below.")
        #         return -1
        #     else:
        #         print("Next node is vertically aligned with the current node.")
        #         if self.direction in directions:
        #             return self.direction
        #         elif len(directions):
        #             return choice(directions)
        #         return self.direction*-1

    # Ghost can get stuck when having to reverse its direction
    def validDirections(self):
        directions = []
        for key in [UP, DOWN, LEFT, RIGHT]:
            if self.LocalValidDirection(key):
                # if key != self.direction * -1:
                directions.append(key)
        if len(directions) == 0:
            directions.append(self.direction * -1)
        return directions
    
    def LocalValidDirection(self, direction):
        if direction is not STOP:
            if self.name in self.node.access[direction]:
                if self.node.neighbors[direction] is not None:
                    return True
        return False
