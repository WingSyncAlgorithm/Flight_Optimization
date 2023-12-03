import pygame
import random

# Define some colors
BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
RED   = (255,   0,   0)

class Block(pygame.sprite.Sprite):
    def __init__(self, color, width, height):
        super().__init__()
        self.image = pygame.Surface([width, height])
        self.image.fill(WHITE)
        self.image.set_colorkey(WHITE)
        pygame.draw.ellipse(self.image, color, [0, 0, width, height])
        # Fetch the rectangle object that has the dimensions of the image
        # Update the position of this object by setting the values of rect.x and rect.y
        self.rect = self.image.get_rect()

def main():
    pygame.init()
    screen_width = 800
    screen_height = 600
    screen = pygame.display.set_mode([800, 600])
    pygame.display.set_caption('Block')
    # This is a list of 'sprites.' Each block in the program is added to this list. 
    # The list is managed by a class called 'Group.'
    block_list = pygame.sprite.Group()
    # This is a list of every sprite. 
    # All blocks and the player block as well.
    all_sprites_list = pygame.sprite.Group()
    #create 50 blocks
    for i in range(50):
        # This represents a block
        block = Block(BLACK, 20, 15)
        # Set a random location for the block
        block.rect.x = random.randrange(screen_width)
        block.rect.y = random.randrange(screen_height)
        # Add the block to the list of objects
        block_list.add(block)
        all_sprites_list.add(block)
 
    # Create a RED player block
    player = Block(RED, 20, 15)
    all_sprites_list.add(player)
 
    # Loop until the user clicks the close button.
    running=True
    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()
    score = 0
    # Set up the font object
    font = pygame.font.Font(None, 50)
    # -------- Main Program Loop -----------
    while running:
        for event in pygame.event.get(): 
            if event.type == pygame.QUIT: 
                running=False
        # Clear the screen
        screen.fill(WHITE)
        # Get the current mouse position. This returns the position
        # as a list of two numbers.
        pos = pygame.mouse.get_pos()
        # Fetch the x and y out of the list,just like we'd fetch letters out of a string.
        # Set the player object to the mouse location
        player.rect.x = pos[0]
        player.rect.y = pos[1]
 
        # See if the player block has collided with anything.
        blocks_hit_list = pygame.sprite.spritecollide(player, block_list, True)
 
        # Check the list of collisions.
        for block in blocks_hit_list:
            score += 1
        # Draw the score to the screen
        #render(text, antialias, color, background=None) -> Surface
        score_text = font.render(f'Score: {score}', True, (0, 255, 0))
        screen.blit(score_text, (10, 10))
        # Draw all the spites
        all_sprites_list.draw(screen)
 
        # Go ahead and update the screen with what we've drawn.
        pygame.display.update()
 
        # Limit to 60 frames per second
        clock.tick(60)
    pygame.quit()

if __name__ == '__main__':    
    main()    