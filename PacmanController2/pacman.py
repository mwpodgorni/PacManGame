import pygame
from pygame.locals import *
from vector import Vector2
from constants import *
from pacmanStatesMethods import *
from entity import Entity
from sprites import PacmanSprites
from random import choice
import sys

class Pacman(Entity):
    def __init__(self, node, nodes, pelletNodes):
        Entity.__init__(self, node )
        self.name = PACMAN    
        self.color = YELLOW
        self.direction = LEFT
        self.setBetweenNodes(LEFT)
        self.alive = True
        self.directionMethod = self.goalDirectionDij
        self.sprites = PacmanSprites(self)
        self.leftoverPellets = []
        self.currentPellet = None
        self.smallestGhostDistance = float('inf')
        self.nodes=nodes
        self.pelletNodes = pelletNodes
        self.states = [SEEK_PELLET, RUN_AWAY, SEEK_GHOST]
        self.myState = SEEK_PELLET

        self.activeNodes = []
        self.activeGoal = None

    def getGhosts(self, ghosts):
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
        self.sprites.update(dt)
        self.position += self.directions[self.direction]*self.speed*dt
        self.updateState()
        if self.myState == SEEK_PELLET:
            print('state == seek pellet')
            seekPellet(self)
        if self.myState == RUN_AWAY:
            print('state == run away')
            runAway(self)
        if self.myState == SEEK_GHOST:
            print('state == seek ghost')
            seekGhost(self)

        if self.overshotTarget():
            self.node = self.target
            directions = self.getValidDirections()
            print('directions', directions)
            direction = self.directionMethod(directions)
            if self.node.neighbors[PORTAL] is not None:
                self.node = self.node.neighbors[PORTAL]
            self.target = self.getNewTarget(direction)
            if self.target is not self.node:
                self.direction = direction
            else:
                self.target = self.getNewTarget(self.direction)

            if self.target is self.node:
                self.direction = STOP
            self.setPosition()

    def updateState(self):
        # priority: 1 - seek ghost, 2 - run away, 3 - seek pellet
        # check seek ghost
        if all(ghost.mode.current == 2 for ghost in self.ghosts):
            self.myState = SEEK_GHOST
        else:
            # check nearest ghost position
            smallest_distance, nearest_ghost = self.getNearestGhost([CHASE, SCATTER])
            if smallest_distance < 3500:
                self.myState = RUN_AWAY
            else:
                self.myState = SEEK_PELLET

    def getNearestGhost(self, states):
        smallest_distance = float('inf')
        nearest_ghost = None
        for ghost in self.ghosts:
            distance = (ghost.position - self.position).magnitudeSquared()
            if distance < smallest_distance and ghost.mode.current in states:
                smallest_distance = distance
                nearest_ghost = ghost
        return smallest_distance, nearest_ghost
    
    
    def getRunAwayDirection(self, directions):
        distances = []
        for direction in directions:
            vec = self.node.position  + self.directions[direction]*TILEWIDTH - self.activeGoal
            distances.append(vec.magnitudeSquared())
        index = distances.index(max(distances))
        return directions[index]
    
    def getValidDirections(self):
        directions = []
        for key in [UP, DOWN, LEFT, RIGHT]:
            if self.validDirection(key):
                directions.append(key)
        if len(directions) == 0:
            directions.append(self.direction * -1)
        return directions

    
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

        pacmanTarget = self.target
        pacmanTarget = self.activeNodes.getVectorFromLUTNode(pacmanTarget)
        path.append(pacmanTarget)
        nextPacmanNode = path[1]
        print('path', path)
        print(directions)
        if pacmanTarget[0] > nextPacmanNode[0] and 2 in directions : #left
            return 2
        if pacmanTarget[0] < nextPacmanNode[0] and -2 in directions : #right
            return -2
        if pacmanTarget[1] > nextPacmanNode[1] and 1 in directions : #up
            return 1
        if pacmanTarget[1] < nextPacmanNode[1] and -1 in directions : #down
            return -1
        else:
            if -1 * self.direction in directions:
                return -1 * self.direction
            else:
                return choice(directions)
        # up 1, down -1, left 2, right -2

    #############
    # Executes Dijkstra from Ghost's previous node as start
    # to pacman's target node as target.
    def getDijkstraPath(self, directions):
        goalNode = (self.activeGoal.x, self.activeGoal.y)
        pacmanTarget = self.target
        pacmanTarget = self.activeNodes.getVectorFromLUTNode(pacmanTarget)

        # previous_nodes, shortest_path = dijkstra(self.nodes, ghostTarget)
        previous_nodes, shortest_path = self.dijkstra_or_a_star(self.activeNodes, pacmanTarget, a_star=True)
        path = []
        node = goalNode
        while node != pacmanTarget:
            path.append(node)
            if node in previous_nodes:
                node = previous_nodes[node]
            else:
                print('while broken')
                break
        path.append(pacmanTarget)
        path.reverse()
        return path


    #########
    # A*
    def heuristic(self, node1, node2):
        # manhattan distance
        return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])


    def dijkstra_or_a_star(self, nodes, start_node, a_star=False):
        unvisited_nodes = list(nodes.costs)
        shortest_path = {}
        previous_nodes = {}

        max_value = sys.maxsize
        for node in unvisited_nodes:
            shortest_path[node] = max_value
        shortest_path[start_node] = 0

        while unvisited_nodes:
            current_min_node = None
            for node in unvisited_nodes:
                if current_min_node == None:
                    current_min_node = node
                elif shortest_path[node] < shortest_path[current_min_node]:
                    current_min_node = node

            neighbors = nodes.getNeighbors(current_min_node)
            for neighbor in neighbors:
                if a_star:
                    tentative_value = shortest_path[current_min_node] + self.heuristic(current_min_node,neighbor) 
                else:
                    tentative_value = shortest_path[current_min_node] + 1
                if neighbor in shortest_path and tentative_value < shortest_path[neighbor]:
                    shortest_path[neighbor] = tentative_value
                    # We also update the best path to the current node
                    previous_nodes[neighbor] = current_min_node
    
            # After visiting its neighbors, we mark the node as "visited"
            unvisited_nodes.remove(current_min_node)
        return previous_nodes, shortest_path