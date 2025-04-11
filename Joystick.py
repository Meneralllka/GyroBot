import pygame
import os

# Initialize pygame and joystick module
pygame.init()
pygame.joystick.init()

# Check if any joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect a gamepad.")
    exit()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Window settings
WIDTH, HEIGHT = 800, 700
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 200, 50)
RED = (200, 50, 50)
BLUE = (0, 100, 255)

# Pygame setup
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption(f"Gamepad: {joystick.get_name()}")

# Load fonts
merriweather_font_path = "Merriweather-Bold.ttf"
roboto_font_path = "Roboto-Bold.ttf"

# Check if font files exist
if not os.path.exists(merriweather_font_path):
    merriweather_font_path = None

if not os.path.exists(roboto_font_path):
    roboto_font_path = None

# Fonts
font_large = pygame.font.Font(merriweather_font_path, 80) if merriweather_font_path else pygame.font.Font(None, 80)
font_normal = pygame.font.Font(roboto_font_path, 50) if roboto_font_path else pygame.font.Font(None, 50)
font_small = pygame.font.Font(roboto_font_path, 40) if roboto_font_path else pygame.font.Font(None, 40)

# Get button and axis count
num_axes = joystick.get_numaxes()

# Mode variable (default: manual)
manual_mode = True


def draw_text(surface, text, x, y, font, color=BLACK):
    """Helper function to draw text on the screen."""
    label = font.render(text, True, color)
    surface.blit(label, (x, y))


def draw_arrow(surface, x, y, direction, active):
    """Draw larger directional arrows with black contour. If active, change color and show label."""
    arrow_size = 160  # Increased from 80 to 160 (2x larger)
    arrow_thickness = 20  # Increased thickness

    directions = {
        "UP": (0, -1),
        "DOWN": (0, 1),
        "LEFT": (-1, 0),
        "RIGHT": (1, 0),
        "UP-RIGHT": (0.7, -0.7),
        "UP-LEFT": (-0.7, -0.7),
        "DOWN-RIGHT": (0.7, 0.7),
        "DOWN-LEFT": (-0.7, 0.7)
    }

    if direction not in directions:
        return

    dx, dy = directions[direction]

    # Arrowhead points
    tip = (x + dx * arrow_size, y + dy * arrow_size)
    left_wing = (x - dy * arrow_thickness, y + dx * arrow_thickness)
    right_wing = (x + dy * arrow_thickness, y - dx * arrow_thickness)

    # Set arrow color based on activation
    arrow_color = BLUE if active else BLACK

    # Draw contour (always black for visibility)
    pygame.draw.polygon(surface, BLACK, [tip, left_wing, right_wing], 20)

    # Draw actual arrow
    pygame.draw.polygon(surface, arrow_color, [tip, left_wing, right_wing])

    # Direction label positions
    direction_text_position = {
        "UP": (x - 40, y - arrow_size - 70),
        "DOWN": (x - 60, y + arrow_size + 30),
        "LEFT": (x - arrow_size - 120, y - 20),
        "RIGHT": (x + arrow_size + 40, y - 20),
        "UP-RIGHT": (x + arrow_size, y - arrow_size - 20),
        "UP-LEFT": (x - arrow_size - 100, y - arrow_size - 20),
        "DOWN-RIGHT": (x + arrow_size, y + arrow_size),
        "DOWN-LEFT": (x - arrow_size - 100, y + arrow_size),
    }

    # Show direction label if active
    if active and direction in direction_text_position:
        draw_text(surface, direction.upper(), *direction_text_position[direction], font_small, BLUE)


def get_active_direction(axis_x, axis_y):
    """Determine which direction is active based on joystick movement."""
    threshold = 0.3

    if axis_y < -threshold and abs(axis_x) < threshold:
        return "UP"
    elif axis_y > threshold and abs(axis_x) < threshold:
        return "DOWN"
    elif axis_x < -threshold and abs(axis_y) < threshold:
        return "LEFT"
    elif axis_x > threshold and abs(axis_y) < threshold:
        return "RIGHT"
    elif axis_x > threshold and axis_y < -threshold:
        return "UP-RIGHT"
    elif axis_x < -threshold and axis_y < -threshold:
        return "UP-LEFT"
    elif axis_x > threshold and axis_y > threshold:
        return "DOWN-RIGHT"
    elif axis_x < -threshold and axis_y > threshold:
        return "DOWN-LEFT"
    return None


def manual_control(axis_x, axis_y):
    """Function to handle manual mode movement."""
    direction = get_active_direction(axis_x, axis_y)
    if direction:
        print(f"Moving {direction}")
    return direction


def auto_control():
    """Function to handle auto mode using computer vision (placeholder)."""
    print("AUTO MODE: Running Computer Vision-based control")

    # === PLACEHOLDER: INSERT YOUR COMPUTER VISION CODE HERE ===

    return None


def main():
    global manual_mode
    running = True
    clock = pygame.time.Clock()

    while running:
        screen.fill(WHITE)
        pygame.event.pump()

        # Read joystick values
        axis_x = joystick.get_axis(0)
        axis_y = joystick.get_axis(1)

        # Switch mode when Button 0 is pressed
        if joystick.get_button(0):
            manual_mode = not manual_mode
            pygame.time.wait(300)

            # Determine behavior based on mode
        if manual_mode:
            mode_text = "MANUAL MODE"
            color = GREEN
            active_direction = manual_control(axis_x, axis_y)
        else:
            mode_text = "AUTO MODE"
            color = RED
            active_direction = auto_control()

        # Draw mode title using Merriweather
        draw_text(screen, mode_text, WIDTH // 2 - 220, 50, font_large, color)

        # Draw all arrows (always visible)
        for direction in ["UP", "DOWN", "LEFT", "RIGHT", "UP-RIGHT", "UP-LEFT", "DOWN-RIGHT", "DOWN-LEFT"]:
            draw_arrow(screen, WIDTH // 2, HEIGHT // 2, direction, active=(direction == active_direction))

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()


if __name__ == "__main__":
    main()
