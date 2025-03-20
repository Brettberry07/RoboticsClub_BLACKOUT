from mod import *
from button import Button
from robot import Robot
from field import Field


# instances
robot = Robot(60, 200, 75, 75, ROBOT_GRAY, 0)

redRingRushButton = Button(2, 30, 190, 50, RED, "Red Ring Rush", lambda: print("Button clicked!"))
redGoalRushButton = Button(2, 140, 190, 50, RED, "Red Goal Rush", lambda: print("Button clicked!"))

blueRingRushButton = Button(2, 250, 190, 50, BLUE, "Blue Ring Rush", lambda: print("Button clicked!"))
blueGoalRushButton = Button(2, 360, 190, 50, BLUE, "Blue Goal Rush", lambda: print("Button clicked!"))

skillsButton = Button(2, 480, 190, 50, GREEN, "Skills", lambda: print("Button clicked!"))

buttons = [redRingRushButton, redGoalRushButton, blueRingRushButton, blueGoalRushButton, skillsButton]

field = Field("default")

robot.rotate(180)  # Initialize robot heading

# Main loop
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            running = False
        # Handle button click events
        if event.type == pygame.MOUSEBUTTONDOWN:
            for button in buttons:
                button.handle_event(event)

    # Fill the screen with white
    screen.fill(WHITE)

    field.draw(field_surface)

    field_surface.blit(field_image, (0, 0))

    # drawing the button sections (bottom portion of the screen)
    pygame.draw.line(screen, RED, (195, 0), (195, HEIGHT), 5)
    
    for button in buttons:
        button.draw(screen)

    robot.draw(field_surface)


    # Update the display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(60)

# Quit Pygame
pygame.quit()
sys.exit()
