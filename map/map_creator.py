import pygame as pg
import numpy as np
import os

# Configuration
ROWS = 40  # Hard-coded số hàng
COLS = 40  # Hard-coded số cột
CELL_SIZE = 20  # Kích thước mỗi ô (pixels)
BORDER = 2

# Colors
BLACK = (0, 0, 0)  # Vật cản (obstacle)
WHITE = (255, 255, 255)  # Không gian trống (free space)
GRAY = (128, 128, 128)  # Grid lines
RED = (255, 0, 0)  # Current cell highlight


class MapCreator:
    def __init__(self):
        pg.init()

        # Window setup
        self.width = COLS * CELL_SIZE + BORDER * 2
        self.height = ROWS * CELL_SIZE + BORDER * 2 + 50  # +50 for status bar
        self.screen = pg.display.set_mode((self.width, self.height))
        pg.display.set_caption("Custom Map Creator - Click: Toggle Obstacle, Space: Save")

        # Font for status
        self.font = pg.font.Font(None, 24)

        # Map data - 0: free space, 1: obstacle
        self.map_data = np.zeros((ROWS, COLS), dtype=int)

        # State
        self.drawing = False
        self.erasing = False
        self.clock = pg.time.Clock()

    def get_cell_from_mouse(self, mouse_pos):
        """Convert mouse position to grid cell coordinates"""
        x, y = mouse_pos
        if x < BORDER or x >= self.width - BORDER:
            return None
        if y < BORDER or y >= self.height - BORDER - 50:
            return None

        col = (x - BORDER) // CELL_SIZE
        row = (y - BORDER) // CELL_SIZE

        if 0 <= row < ROWS and 0 <= col < COLS:
            return (row, col)
        return None

    def draw_grid(self):
        """Draw the grid and map"""
        self.screen.fill(WHITE)

        # Draw map cells
        for row in range(ROWS):
            for col in range(COLS):
                x = col * CELL_SIZE + BORDER
                y = row * CELL_SIZE + BORDER

                # Cell color based on value
                color = BLACK if self.map_data[row, col] == 1 else WHITE
                pg.draw.rect(self.screen, color, (x, y, CELL_SIZE, CELL_SIZE))

                # Grid lines
                pg.draw.rect(self.screen, GRAY, (x, y, CELL_SIZE, CELL_SIZE), 1)

        # Highlight current cell under mouse
        mouse_pos = pg.mouse.get_pos()
        cell = self.get_cell_from_mouse(mouse_pos)
        if cell:
            row, col = cell
            x = col * CELL_SIZE + BORDER
            y = row * CELL_SIZE + BORDER
            pg.draw.rect(self.screen, RED, (x, y, CELL_SIZE, CELL_SIZE), 2)

    def draw_status(self):
        """Draw status bar"""
        status_y = ROWS * CELL_SIZE + BORDER + 10

        # Count obstacles and free spaces
        obstacles = np.sum(self.map_data == 1)
        free_spaces = np.sum(self.map_data == 0)
        total_cells = ROWS * COLS
        obstacle_percentage = (obstacles / total_cells) * 100

        # Status text
        status_text = [
            f"Map Size: {COLS}x{ROWS} ({total_cells} cells)",
            f"Obstacles: {obstacles} ({obstacle_percentage:.1f}%) | Free: {free_spaces}",
            "Controls: Left Click = Add Obstacle | Right Click = Remove | Space = Save"
        ]

        for i, text in enumerate(status_text):
            color = BLACK if i < 2 else RED
            text_surface = self.font.render(text, True, color)
            self.screen.blit(text_surface, (10, status_y + i * 20))

    def save_map(self):
        """Save map to file"""
        # Create output directory if it doesn't exist
        if not os.path.exists('map/custom'):
            os.makedirs('map/custom')

        # Find next available filename
        filename_base = 'map/custom/custom_map'
        counter = 1
        while os.path.exists(f"{filename_base}_{counter:02d}.txt"):
            counter += 1

        filename = f"{filename_base}_{counter:02d}.txt"

        try:
            with open(filename, 'w') as f:
                # Write header (cols rows)
                f.write(f"{COLS} {ROWS}\n")

                # Write map data
                for row in range(ROWS):
                    row_data = []
                    for col in range(COLS):
                        row_data.append(str(self.map_data[row, col]))
                    f.write(" ".join(row_data) + "\n")

            print(f"✅ Map saved successfully: {filename}")

            # Show confirmation on screen briefly
            self.show_save_confirmation(filename)

        except Exception as e:
            print(f"❌ Error saving map: {e}")

    def show_save_confirmation(self, filename):
        """Show save confirmation overlay"""
        overlay = pg.Surface((self.width, self.height))
        overlay.set_alpha(128)
        overlay.fill(BLACK)
        self.screen.blit(overlay, (0, 0))

        # Confirmation text
        text_lines = [
            "MAP SAVED SUCCESSFULLY!",
            f"File: {filename}",
            "Press any key to continue..."
        ]

        font_big = pg.font.Font(None, 48)
        font_small = pg.font.Font(None, 24)

        for i, line in enumerate(text_lines):
            font = font_big if i == 0 else font_small
            color = (0, 255, 0) if i == 0 else WHITE
            text_surface = font.render(line, True, color)
            text_rect = text_surface.get_rect(center=(self.width // 2, self.height // 2 + i * 40))
            self.screen.blit(text_surface, text_rect)

        pg.display.flip()

        # Wait for keypress
        waiting = True
        while waiting:
            for event in pg.event.get():
                if event.type == pg.KEYDOWN or event.type == pg.QUIT:
                    waiting = False

    def handle_mouse_drawing(self, mouse_pos, mouse_buttons):
        """Handle mouse drawing/erasing"""
        cell = self.get_cell_from_mouse(mouse_pos)
        if not cell:
            return

        row, col = cell

        # Left click: add obstacle
        if mouse_buttons[0]:  # Left mouse button
            self.map_data[row, col] = 1

        # Right click: remove obstacle
        elif mouse_buttons[2]:  # Right mouse button
            self.map_data[row, col] = 0

    def run(self):
        """Main application loop"""
        running = True

        while running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    running = False

                elif event.type == pg.KEYDOWN:
                    if event.key == pg.K_SPACE:
                        self.save_map()
                    elif event.key == pg.K_c:
                        # Clear map
                        self.map_data.fill(0)
                        print("🧹 Map cleared")
                    elif event.key == pg.K_f:
                        # Fill map (all obstacles)
                        self.map_data.fill(1)
                        print("🟫 Map filled with obstacles")

            # Handle continuous mouse drawing
            mouse_buttons = pg.mouse.get_pressed()
            if mouse_buttons[0] or mouse_buttons[2]:  # Left or right click
                mouse_pos = pg.mouse.get_pos()
                self.handle_mouse_drawing(mouse_pos, mouse_buttons)

            # Draw everything
            self.draw_grid()
            self.draw_status()
            pg.display.flip()
            self.clock.tick(60)  # 60 FPS

        pg.quit()


def main():
    print("🎮 Custom Map Creator")
    print(f"📏 Map Size: {COLS} x {ROWS}")
    print("🖱️  Controls:")
    print("   - Left Click: Add obstacles")
    print("   - Right Click: Remove obstacles")
    print("   - Space: Save map")
    print("   - C: Clear all")
    print("   - F: Fill all")
    print("   - Close window: Exit")
    print("\n🚀 Starting map creator...")

    creator = MapCreator()
    creator.run()


if __name__ == "__main__":
    main()