from mod import pygame, time

class Button:
    def __init__(self, x, y, width, height, color, text, command):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color
        self.text = text
        self.command = command
        self.font = pygame.font.Font(None, 36)

    def draw(self, screen):
        pygame.draw.rect(screen, self.color, (self.x, self.y, self.width, self.height))
        text_surface = self.font.render(self.text, True, (255, 255, 255))
        text_rect = text_surface.get_rect(center=(self.x + self.width // 2, self.y + self.height // 2))
        screen.blit(text_surface, text_rect)

    def is_clicked(self, mouse_pos):
        return self.x <= mouse_pos[0] <= self.x + self.width and self.y <= mouse_pos[1] <= self.y + self.height

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.is_clicked(event.pos):
                self.command()
                time.sleep(0.5) # prevents multiple clicks

# Example usage:
    # def button_command():
    #     print("Button clicked!")

    # button = Button(300, 250, 200, 50, (0, 128, 0), "Click Me", button_command)
    # button = Button(300, 250, 200, 50, (0, 128, 0), "Click Me", lambda: print("Button clicked!"))