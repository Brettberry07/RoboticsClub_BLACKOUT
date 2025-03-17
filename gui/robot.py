from mod import pygame

class Robot:
    def __init__(self, x, y, w, h, color, heading):
        """
        Initializes a Robot instance.

        :param x: Initial x-coordinate of the robot.
        :param y: Initial y-coordinate of the robot.
        :param w: Width of the robot.
        :param h: Height of the robot.
        :param color: Color of the robot (RGB tuple).
        :param heading: Initial orientation angle of the robot in degrees.
        """
        
        # Initialize the robot's attributes
        self.x = x
        self.y = y
        self.width = w
        self.height = h
        self.color = color
        self.heading = heading

        # Create a rectangle for the robot and set its position
        self.rect = pygame.Rect(x, y, w, h)
        self.rect.center = (self.x, self.y)
        self.image = pygame.Surface((w, h))
        self.image.fill(color)


        # Store the original image and rect for rotation
        self.original_image = self.image.copy()
        self.original_rect = self.rect.copy()


    def draw(self, screen):
        """
        Draws the robot on the given screen.

        :param screen: The Pygame surface on which to draw the robot.
        """
        
        # Rotate the image based on the heading
        rotated_image = pygame.transform.rotate(self.original_image, -self.heading)
        rotated_rect = rotated_image.get_rect(center=self.rect.center)
        screen.blit(rotated_image, rotated_rect.topleft)
        
        # Update the rect to match the new position
        self.rect = rotated_rect    


    def move_robot(self, dx, dy):
        """
        Moves the robot to a new position based on the current position and the specified deltas.

        :param self: The instance of the Robot class.
        :param dx: Change in x-coordinate (delta x).
        :param dy: Change in y-coordinate (delta y).
        """

        self.x += dx
        self.y += dy
        self.rect.center = (self.x, self.y)
        

    def rotate_robot(current_angle, delta_angle):
        """
        Rotates the robot by a specified angle.

        :param current_angle: Current orientation angle of the robot in degrees.
        :param delta_angle: Change in angle (delta angle) in degrees.
        :return: New orientation angle of the robot.
        """
        new_angle = (current_angle + delta_angle) % 360
        return new_angle
        
