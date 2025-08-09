import sys
import pygame as pg
import numpy as np
import sys
import os
from datetime import datetime

# Constants
GRID_SIZE = 10
CELL_SIZE = 60
WINDOW_WIDTH = GRID_SIZE * CELL_SIZE
WINDOW_HEIGHT = GRID_SIZE * CELL_SIZE + 150  # Extra space for info

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (128, 128, 128)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
LIGHT_BLUE = (173, 216, 230)


class BWaveFig5Cosplay:
    def __init__(self):
        pg.init()
        self.screen = pg.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pg.display.set_caption("BWave Figure 5 Cosplay - Weighted Map Generation")

        # Grid states: 0=free, 1=obstacle, 'v'=visited
        self.grid_G = np.zeros((GRID_SIZE, GRID_SIZE), dtype=object)

        # Mask M (for Boustrophedon left-to-right priority)
        self.mask_M = None

        # Weighted map W_G
        self.weighted_map_WG = None

        # Current display mode
        self.display_mode = "GRID"  # "GRID", "MASK", "WEIGHTED"

        # Font for displaying numbers
        self.font = pg.font.Font(None, 24)
        self.title_font = pg.font.Font(None, 36)
        if not os.path.exists('screenshots'):
            os.makedirs('screenshots')

    def create_mask_M(self):
        """Create mask M following BWave paper: M[i][j] = col_count - j"""
        self.mask_M = np.zeros((GRID_SIZE, GRID_SIZE))

        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                self.mask_M[i][j] = GRID_SIZE - j  # Left-to-right priority

        print("✅ Mask M generated!")
        print("Mask M:")
        print(self.mask_M.astype(int))

    def create_weighted_map_WG(self):
        """Create weighted map W_G following BWave equation (2)"""
        if self.mask_M is None:
            print("❌ Please generate mask M first!")
            return

        self.weighted_map_WG = np.zeros((GRID_SIZE, GRID_SIZE))

        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                if self.grid_G[i][j] == 'v':  # visited
                    self.weighted_map_WG[i][j] = 0
                elif self.grid_G[i][j] == 1:  # obstacle
                    self.weighted_map_WG[i][j] = -1
                else:  # unvisited (free)
                    self.weighted_map_WG[i][j] = self.mask_M[i][j]

        print("✅ Weighted Map W_G generated!")
        print("Weighted Map W_G:")
        print(self.weighted_map_WG.astype(int))

    def save_grid_screenshot(self):
        """Save only the grid area (without instructions) as screenshot"""
        # Create surface for grid area only
        grid_surface = pg.Surface((GRID_SIZE * CELL_SIZE, GRID_SIZE * CELL_SIZE))

        # Draw grid on separate surface
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                x = j * CELL_SIZE
                y = i * CELL_SIZE
                rect = pg.Rect(x, y, CELL_SIZE, CELL_SIZE)

                # Same drawing logic as main draw_grid but on grid_surface
                if self.display_mode == "GRID":
                    if self.grid_G[i][j] == 1:
                        color = BLACK
                        text = "OBS"
                    elif self.grid_G[i][j] == 'v':
                        color = GREEN
                        text = "VIS"
                    else:
                        color = WHITE
                        text = "0"

                elif self.display_mode == "MASK":
                    if self.mask_M is None:
                        color = GRAY
                        text = "?"
                    else:
                        value = self.mask_M[i][j]
                        intensity = int(255 * (value / GRID_SIZE))
                        color = (255 - intensity, 255 - intensity, 255)
                        text = str(int(value))

                elif self.display_mode == "WEIGHTED":
                    if self.weighted_map_WG is None:
                        color = GRAY
                        text = "?"
                    else:
                        value = self.weighted_map_WG[i][j]
                        if value == -1:
                            color = BLACK
                            text = "-1"
                        elif value == 0:
                            color = GREEN
                            text = "0"
                        else:
                            intensity = int(255 * (value / GRID_SIZE))
                            color = (255 - intensity, 255 - intensity, 255)
                            text = str(int(value))

                # Draw cell on grid surface
                pg.draw.rect(grid_surface, color, rect)
                pg.draw.rect(grid_surface, BLACK, rect, 2)

                # Draw text
                text_surface = self.font.render(text, True, RED if color == WHITE else WHITE)
                text_rect = text_surface.get_rect(center=rect.center)
                grid_surface.blit(text_surface, text_rect)

        # Generate filename with timestamp and mode
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"screenshots/bwave_fig5_{self.display_mode.lower()}_{timestamp}.png"

        # Save screenshot
        pg.image.save(grid_surface, filename)
        print(f"📸 Screenshot saved: {filename}")

    def handle_click(self, pos):
        """Handle mouse click to toggle obstacles"""
        mouse_x, mouse_y = pos

        # Convert to grid coordinates
        grid_j = mouse_x // CELL_SIZE
        grid_i = mouse_y // CELL_SIZE

        if 0 <= grid_i < GRID_SIZE and 0 <= grid_j < GRID_SIZE:
            # Toggle obstacle
            if self.grid_G[grid_i][grid_j] == 1:
                self.grid_G[grid_i][grid_j] = 0  # Remove obstacle
            elif self.grid_G[grid_i][grid_j] == 0:
                self.grid_G[grid_i][grid_j] = 1  # Add obstacle

            # Update weighted map if it exists
            if self.weighted_map_WG is not None:
                self.create_weighted_map_WG()

            print(f"Toggled cell ({grid_i}, {grid_j})")

    def draw_grid(self):
        """Draw the current grid based on display mode"""
        self.screen.fill(WHITE)

        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                x = j * CELL_SIZE
                y = i * CELL_SIZE
                rect = pg.Rect(x, y, CELL_SIZE, CELL_SIZE)

                # Determine cell color and value based on mode
                if self.display_mode == "GRID":
                    if self.grid_G[i][j] == 1:
                        color = BLACK  # Obstacle
                        text = "OBS"
                    elif self.grid_G[i][j] == 'v':
                        color = GREEN  # Visited
                        text = "VIS"
                    else:
                        color = WHITE  # Free
                        text = "0"

                elif self.display_mode == "MASK":
                    if self.mask_M is None:
                        color = GRAY
                        text = "?"
                    else:
                        # Color gradient based on mask value
                        value = self.mask_M[i][j]
                        intensity = int(255 * (value / GRID_SIZE))
                        color = (255 - intensity, 255 - intensity, 255)
                        text = str(int(value))

                elif self.display_mode == "WEIGHTED":
                    if self.weighted_map_WG is None:
                        color = GRAY
                        text = "?"
                    else:
                        value = self.weighted_map_WG[i][j]
                        if value == -1:
                            color = BLACK  # Obstacle
                            text = "-1"
                        elif value == 0:
                            color = GREEN  # Visited
                            text = "0"
                        else:
                            # Positive weights - color gradient
                            intensity = int(255 * (value / GRID_SIZE))
                            color = (255 - intensity, 255 - intensity, 255)
                            text = str(int(value))

                # Draw cell
                pg.draw.rect(self.screen, color, rect)
                pg.draw.rect(self.screen, BLACK, rect, 2)  # Border

                # Draw text
                text_surface = self.font.render(text, True, RED if color == WHITE else WHITE)
                text_rect = text_surface.get_rect(center=rect.center)
                self.screen.blit(text_surface, text_rect)

        # Draw title and instructions
        title_y = GRID_SIZE * CELL_SIZE + 10

        title_text = ""
        if self.display_mode == "GRID":
            title_text = "Grid G - Click to toggle obstacles"
        elif self.display_mode == "MASK":
            title_text = "Mask M - Boustrophedon Priority Pattern"
        elif self.display_mode == "WEIGHTED":
            title_text = "Weighted Map W_G = Apply Mask to Grid"

        title_surface = self.title_font.render(title_text, True, BLACK)
        self.screen.blit(title_surface, (10, title_y))

        # Status indicator
        status_y = title_y + 30
        status_text = f"Step: "
        if self.mask_M is None:
            status_text += "1 (Create obstacles) → Press SPACE"
        elif self.weighted_map_WG is None:
            status_text += "2 (Mask generated) → Press SPACE again"
        else:
            status_text += "3 (Complete) → Press TAB to cycle views"

        status_surface = pg.font.Font(None, 24).render(status_text, True, BLUE)
        self.screen.blit(status_surface, (10, status_y))

        # Instructions
        inst_y = status_y + 25
        instructions = [
            "Left Click: Toggle obstacle",
            "Space: Generate Mask M / Weighted Map W_G",
            "Tab: Cycle display modes",
            "R: Reset all",
            "ESC: Exit",
            "S: Save grid screenshot"
        ]

        for i, inst in enumerate(instructions):
            inst_surface = pg.font.Font(None, 20).render(inst, True, BLACK)
            self.screen.blit(inst_surface, (10, inst_y + i * 15))

    def run(self):
        """Main game loop"""
        clock = pg.time.Clock()
        running = True
        f4_count = 0

        print("🎮 BWave Figure 5 Cosplay Started!")
        print("📋 Instructions:")
        print("   - Left Click: Toggle obstacles")
        print("   - Space: Generate Mask M / Weighted Map W_G")
        print("   - Tab: Cycle display modes")
        print("   - R: Reset all")
        print("   - S: Save grid screenshot")
        print("   - ESC: Exit")
        print()

        while running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    running = False

                elif event.type == pg.MOUSEBUTTONDOWN:
                    if event.button == 1:  # Left click
                        self.handle_click(event.pos)

                elif event.type == pg.KEYDOWN:
                    if event.key == pg.K_ESCAPE:
                        running = False

                    elif event.key == pg.K_SPACE:
                        f4_count += 1

                        if f4_count == 1:
                            # First Space: Generate Mask
                            self.create_mask_M()
                            self.display_mode = "MASK"

                        elif f4_count == 2:
                            # Second Space: Generate Weighted Map
                            self.create_weighted_map_WG()
                            self.display_mode = "WEIGHTED"

                        else:
                            # Reset cycle
                            f4_count = 0
                            self.display_mode = "GRID"

                    elif event.key == pg.K_TAB:
                        # Cycle display modes
                        modes = ["GRID", "MASK", "WEIGHTED"]
                        current_index = modes.index(self.display_mode)
                        next_index = (current_index + 1) % len(modes)
                        self.display_mode = modes[next_index]

                    elif event.key == pg.K_r:
                        # Reset everything
                        self.grid_G = np.zeros((GRID_SIZE, GRID_SIZE), dtype=object)
                        self.mask_M = None
                        self.weighted_map_WG = None
                        self.display_mode = "GRID"
                        f4_count = 0
                        print("🔄 Reset all data!")
                    elif event.key == pg.K_s:
                        # Save grid screenshot
                        self.save_grid_screenshot()

            # Draw everything
            self.draw_grid()
            pg.display.flip()
            clock.tick(60)

        pg.quit()
        sys.exit()


if __name__ == "__main__":
    app = BWaveFig5Cosplay()
    app.run()