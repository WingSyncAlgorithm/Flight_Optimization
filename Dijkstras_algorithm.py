import sys, pygame
import numpy as np
import math


class Dijkstra:
    def __init__(self,obstacles_x,obstacles_y,stepsize):
        """
        Initialize map for a star planning

        obstacles_x: x position list of Obstacles [m]
        obstacles_y: y position list of Obstacles [m]
        stepsize: grid resolution [m]
        """
        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None
        
        self.robot_radius = None
        self.stepsize=stepsize
        self.calc_obstacle_map(obstacles_x, obstacles_y)
        self.motion = self.get_motion_model()

    class Node:
            def __init__(self, x, y, cost, parent_index):
                self.x = x  # index of grid
                self.y = y  # index of grid
                self.cost = cost
                # self.grid = grid
                self.parent_index = parent_index  # index of previous Node

            def __str__(self):
                #when using print, it will print out the string below
                return str(self.x) + "," + str(self.y) + "," + str(
                    self.cost) + "," + str(self.parent_index)
            
    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.stepsize)

    def calc_index(self, node):
        '''
            return node index by
            1->2->3
            4->5->6
        '''
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)    
    
    def planning(self, start_x, start_y, goal_x, goal_y):
        """
        dijkstra path search

        input:
            start_x: start x position [m]
            start_y: start y position [m]
            goal_x: goal x position [m]
            goal_x: goal x position [m]

        output:
            return_x: x position list of the final path
            return_y: y position list of the final path
        """
        start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
                                    self.calc_xy_index(start_y, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
                                    self.calc_xy_index(goal_y, self.min_y), 0.0, -1)
        
        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node

        while True:
            #pick the least cost node as current
            less_cost_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[less_cost_id]
            #reach goal
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            ''' Remove the item on open set '''
            del open_set[less_cost_id]
            
            ''' Add it to the closed set '''
            closed_set[less_cost_id] = current

            ''' expand search grid based on motion model'''
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, less_cost_id)
                node_id = self.calc_index(node)

                if node_id in closed_set:
                    continue

                if not self.verify_node(node):
                    continue

                if node_id not in open_set:
                    # Discover a new node
                    open_set[node_id] = node  
                else:
                    if open_set[node_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        open_set[node_id] = node
        
        return_x, return_y = self.calc_final_path(goal_node, closed_set)

        return return_x, return_y
    
    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        return_x, return_y = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            return_x.append(self.calc_position(n.x, self.min_x))
            return_y.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        return return_x, return_y
    
    def calc_position(self, index, minp):
        pos = index * self.stepsize + minp
        return pos
    
    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)
        #cross boundaries
        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False
        #cross obstacles
        if self.obstacle_map[node.x][node.y]:
            return False
        return True
    
    @staticmethod
    def get_motion_model():
        ''' motion model for obstacle '''
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
    
    def calc_obstacle_map(self, ox, oy):

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = []
        ''' Iterate over the x, y indices and setting then false '''
        for _ in range(self.x_width):
            row = []
            for _ in range(self.y_width):
                row.append(False)
            self.obstacle_map.append(row)
        ''' Iterate over x indices '''
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                ''' Iterate over obstacles' x and y coordinates '''
                for iox, ioy in zip(ox, oy):
                    ''' Calculate Euclidean distance between obstacle and current point '''
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

class Block(pygame.sprite.Sprite):
    #this class define a pygame rectangle shape sprite 
    def __init__(self, width, height,xpos,ypos,color=(255,255,255)):
        super().__init__()#=pygame.sprite.Sprite.__init__(self)
        self.rect=pygame.Rect(xpos,ypos,width,height)
        self.color=color
        self.image = pygame.Surface([width, height])
        self.image.fill(color)

class Circle(pygame.sprite.Sprite):
    #this class define a circle shape sprite
    def __init__(self, color, radius,pos):
        super().__init__()
        self.image = pygame.Surface([2*radius, 2*radius])
        self.image.fill((0,0,0))
        self.image.set_colorkey((0,0,0))
        self.pos=pos
        #When blitting this Surface onto a destination, any pixels that have the same color as 
        # the colorkey will be transparent.
        pygame.draw.circle(self.image, color,(radius,radius),radius)
        # Fetch the rectangle object that has the dimensions of the image
        # Update the position of this object by setting the values of rect.x and rect.y
        self.rect = self.image.get_rect()
        self.rect.centerx = pos[0]
        self.rect.centery = pos[1]
        
def whether_quit():
    #this function use to determine if user push the close bottom of the window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

if __name__ == '__main__': 
    pygame.init()
    pygame.display.set_caption('RRTstar')
    SCREEN_WIDTH=800
    SCREEN_HIGHT=600
    screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HIGHT))
    screen.fill((0,0,0))
    running=True
        #---------------------setting start and end point
    x0,y0=50,50
    START_P=np.array([x0,y0])
    x_end,y_end=550,550
    END_P=np.array([x_end,y_end])
    ORIGION=Circle((0,255,0),10,START_P)
    END_POINT=Circle((255,0,0),10,END_P)
    point_list = pygame.sprite.Group()
    point_list.add(ORIGION)
    point_list.add(END_POINT)
    #---------------------setting block 
    BLOCK1=Block(100,200,150,0)
    BLOCK2=Block(100,200,300,400)
    BLOCK3=Block(300,100,350,250)
    block_list = pygame.sprite.Group()
    block_list.add(BLOCK1)
    block_list.add(BLOCK2)
    idx=0
    while running:
        screen.fill((0,0,0))
        whether_quit()