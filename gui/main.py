import pygame
import sys
import math

# Initialize Pygame
pygame.init()

# Set up the display
WIDTH, HEIGHT = 800, 600

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Simple Pygame Window")

# Set up colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Main loop

running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Fill the screen with white
    screen.fill(WHITE)

    # Draw a red rectangle
    pygame.draw.rect(screen, RED, (100, 100, 200, 100))

    # Draw a green circle
    pygame.draw.circle(screen, GREEN, (400, 300), 50)

    # Draw a blue line
    pygame.draw.line(screen, BLUE, (600, 100), (700, 500), 5)

    # Update the display
    pygame.display.flip()
    # Cap the frame rate
    pygame.time.Clock().tick(60)
    
# Quit Pygame
pygame.quit()
sys.exit()
# End of the program
