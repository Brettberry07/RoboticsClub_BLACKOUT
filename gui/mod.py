import pygame
import threading
import sys
import time

# Initialize Pygame
pygame.init()

# Set up the clock
clock = pygame.time.Clock()

# Set up the display
WIDTH, HEIGHT = 800,600  # 1 foot = 50 pixels

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Pathing GUI")

# Making a subscreen for the field
field_surface = screen.subsurface((200, 0, 600, 600))

# Set up colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# set up paths to routes
current_route = None

RED_RING_RUSH = "paths/redRingRush.txt"
RED_GOAL_RUSH = "paths/redGoalRush.txt"

BLUE_RING_RUSH = "paths/blueRingRush.txt"
BLUE_GOAL_RUSH = "paths/blueGoalRush.txt"

