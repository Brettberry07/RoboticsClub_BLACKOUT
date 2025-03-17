from mod import *
from button import Button
from robot import Robot


# instances
robot = Robot(400, 300, 50, 30, BLUE, 0)
button = Button(300, 250, 200, 50, GREEN, "Click Me", lambda: print("Button clicked!"))

# Main loop
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            running = False
        # Handle button click events
        if event.type == pygame.MOUSEBUTTONDOWN:
            button.handle_event(event)

    # Fill the screen with white
    screen.fill(WHITE)
    
    button.draw(screen)
    robot.draw(screen)

    # Update the display
    pygame.display.flip()
    
    # Cap the frame rate
    clock.tick(60)

# Quit Pygame
pygame.quit()
sys.exit()
