import pygame
from pygame.locals import *
from vector import Vector2
from constants import *
from entity import Entity
from sprites import PacmanSprites
from algorithms import dijkstra_or_a_star
from random import choice

class Pacman(Entity):
    def __init__(self, node, nodes):
        Entity.__init__(self, node )
        self.name = PACMAN    
        self.color = YELLOW
        self.direction = STOP
        self.setBetweenNodes(LEFT)
        self.alive = True
        self.sprites = PacmanSprites(self)
        self.goal = self.node
        self.states = [SEEK_PELLET, RUN_AWAY, SEEK_GHOST]
        self.myState = SEEK_PELLET
        self.nodes=nodes
        self.pellets=[]
        print('pacman init')
        print(f'node:{self.node.position}')
        print(f'position:{self.position}')


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
        # NON ON NO NO NO
        # Copy ghost setup from session3
        self.sprites.update(dt)
        self.position += self.directions[self.direction]*self.speed*dt
        if self.overshotTarget():
            self.node = self.goal
            # directions = self.validDirections()
            # direction = self.directionMethod(directions)
            # self.target = self.getNewTarget(direction)
            # if self.target is not self.node:
            #     self.direction = direction
            # else:
            #     self.target = self.getNewTarget(self.direction)
            # self.setPosition()
        print('----------------------------')
        # print(f'Slef POS:{self.position}')
        self.updateState()

        if self.myState == SEEK_PELLET:
            self.seekPellet()


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
        # print(f"p:{len(pelletList)}")
        self.pellets = pelletList
        for pellet in pelletList:
            if self.collideCheck(pellet):
                print('EAT PELLETS')
                self.goal=None
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

    def seekPellet(self):
        if self.goal is not None:
            directions = self.validDirections()
            self.direction=self.goalDirection(directions)
        else:
            self.seekNewPellet()

    def seekNewPellet(self):
        if len(self.pellets) > 0:
            # Get the node in the current direction
            # print(f'node:{self.node.position}')
            next_node = self.node.neighbors[self.direction] if self.direction in self.node.neighbors else None
            # Check if there is a pellet on the next node in the same direction
            if next_node and self.pelletExists(next_node.position):
            # # Continue going in the same direction to eat the next pellet
                # print(f"next_node pos:{next_node.position}")            
                return
            nearestPellet = self.getNewNearestPellet()
            self.goal = self.nodes.getNearestNode(nearestPellet.position)
            # self.getShortestPath()                   
        else:
            self.direction=STOP

        # Executes Dijkstra from Ghost's previous node as start 
        # to pacman's target node as target.
    def getPath(self, directions):
        lastPacmanNode = self.goal
        lastPacmanNode = self.nodes.getVectorFromLUTNode(lastPacmanNode)
        ghostTarget = self.target
        ghostTarget = self.nodes.getVectorFromLUTNode(ghostTarget)
        previous_nodes, shortest_path = dijkstra_or_a_star(self.nodes, ghostTarget, a_star=True)
        path = []
        node = lastPacmanNode
        while node != ghostTarget:
            path.append(node)
            node = previous_nodes[node]
        path.append(ghostTarget)
        path.reverse()
        return path
        
    # Chooses direction in which to turn based on the dijkstra
    # returned path
    def goalDirection(self, directions):
        print("goalDirection")
        path = self.getPath(directions)
        # print(path)
        ghostTarget = self.goal
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
            return choice(directions)
        # up 1, down -1, left 2, right -2

    def pelletExists(self, position):
        for pellet in self.pellets:
            if pellet.position == position:
                return True
        return False
    
    def getNewNearestPellet(self):
        # print(f'pellets {self.pellets}')
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
        
    def getShortestPath(self):
        print(f'self.node:{self.node.position}')
        print(f'self.goal:{self.goal.position}')
        # print(f'getShortestPath:{self.nodes, self.node}')
        # target = self.nodes.getVectorFromLUTNode(self.node)
        # print(f'target:{target}')
        # for node in self.nodes.getListOfNodesVector():
            # print(f'pos: {node}')
        # shortest = dijkstra_or_a_star(self.nodes, target, True)
        # print(f'shortest:{shortest}')

    def updateState(self):
        if False:
            print('updateState')