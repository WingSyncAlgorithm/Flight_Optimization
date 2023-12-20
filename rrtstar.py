import sys, pygame
import random
import numpy as np

class Block(pygame.sprite.Sprite):
    def __init__(self, width, height,xpos,ypos,color=(255,255,255)):
        super().__init__()#=pygame.sprite.Sprite.__init__(self)
        self.rect=pygame.Rect(xpos,ypos,width,height)
        self.color=color
        self.image = pygame.Surface([width, height])
        self.image.fill(color)

class Line(pygame.sprite.Sprite):
    def __init__(self,start,end,color=(100,100,244),width=1):
        super().__init__()
        self.start=start
        self.end=end
        self.color=color
        self.rect=pygame.Rect(0,0,0,0)
        self.width=width
    def draw(self,window):
        self.rect=pygame.draw.line(window,self.color,self.start,self.end,width=self.width)

class Circle(pygame.sprite.Sprite):
    def __init__(self, color, radius,pos):
        super().__init__()
        self.image = pygame.Surface([2*radius, 2*radius])
        self.image.fill((0,0,0))
        self.image.set_colorkey((0,0,0))
        #When blitting this Surface onto a destination, any pixels that have the same color as 
        # the colorkey will be transparent.
        pygame.draw.circle(self.image, color,(radius,radius),radius)
        # Fetch the rectangle object that has the dimensions of the image
        # Update the position of this object by setting the values of rect.x and rect.y
        self.rect = self.image.get_rect()
        self.rect.centerx = pos[0]
        self.rect.centery = pos[1]


def if_collide(new_point_x,new_point_y,block_list):
    bool=False
    for block in block_list:
        if new_point_x > block.rect.left and  new_point_x < block.rect.right \
        and new_point_y < block.rect.bottom and new_point_y > block.rect.top:
            bool=True
    return bool

def find_most_closed_point(point_list,point_in):
    norm_min = max(np.abs(point_list[0]-point_in))
    point_out = np.array([0,0])
    for pt in point_list:
        inf_norm = max(np.abs(pt-point_in))
        if inf_norm <= norm_min:
            norm_min = inf_norm
            point_out = pt
    return point_out

def RRTstar(block,start,end,step_size,max_itertime,screen,point_list):
    path_tree = []
    path_tree.append(start)
    for i in range(max_itertime):
        x_rand = random.randrange(screen.get_width())
        y_rand = random.randrange(screen.get_height())
        p_rand=np.array([x_rand,y_rand])
        p_close = find_most_closed_point(path_tree,p_rand)
        direction = p_rand-p_close
        unit_vector = direction / np.linalg.norm(direction)
        p_new = p_close+step_size*unit_vector
        #print(p_rand,p_close,p_new)
        if not if_collide(p_new[0],p_new[1],block):
            path_tree.append(p_new)
            draw_path(screen,point_list,p_new,p_close)
            if max(np.abs(end-p_new))<=step_size:
                l_new=Line(p_new,end)
                l_new.draw(screen)
                pygame.display.update()
                break
    return path_tree

def draw_path(screen,point_list,p_new,p_close):
    clock = pygame.time.Clock()
    c_new=Circle((0,0,255),5,p_new)
    point_list.add(c_new)
    l_new=Line(p_close,p_new)
    point_list.draw(screen)
    l_new.draw(screen)
    clock.tick(20) #frames per second
    pygame.display.update()

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
    # block_list.add(BLOCK3)
    #---------------------rrt and display
    idx=0
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
        screen.fill((0,0,0))
        block_list.draw(screen)
        if idx<1:
            path=RRTstar(block_list,START_P,END_P,30,1000,screen,point_list)
            path.pop(0)
            print(len(path))
            idx=1       