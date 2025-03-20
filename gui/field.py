from mod import pygame, LIGHT_GRAY, DARK_GRAY

class Field:
    def __init__(self, type):
        self.type = type
    
    def draw(self, screen):
        x, y = 0, 0
        w, h = 100, 100

        current_color = LIGHT_GRAY

        for i in range(6):
            x = 0
            for j in range(6):
                if (i + j) % 2 == 0: # checkboard pattern for colors
                    current_color = LIGHT_GRAY
                else:
                    current_color = DARK_GRAY

                # Draw the square on the field surface
                pygame.draw.rect(screen, current_color, (x, y, w, h))
                x += w
            y += h
        
        if self.type == "skills":
            # Draw the skills placements
            pass

        elif self.type == "match":
            # draw the match placements
            pass
