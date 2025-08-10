import pygame
import sys
import os
from datetime import datetime

# Khởi tạo Pygame
pygame.init()

# Cấu hình map
COLS = 16
ROWS = 16
CELL_SIZE = 30  # Kích thước mỗi ô (pixels)

# Kích thước cửa sổ
WINDOW_WIDTH = COLS * CELL_SIZE
WINDOW_HEIGHT = ROWS * CELL_SIZE + 50  # +50 cho thanh thông tin

# Màu sắc
WHITE = (255, 255, 255)  # Free space (0)
BLACK = (0, 0, 0)  # Obstacle (1)
GRAY = (128, 128, 128)  # Grid lines
RED = (255, 0, 0)  # Text color
GREEN = (0, 255, 0)  # Success message


class MapEditor:
    def __init__(self):
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Coverage Path Planning Map Editor")

        # Khởi tạo map - tất cả đều là free space (0)
        self.grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]

        # Font cho text
        self.font = pygame.font.Font(None, 24)

        # Trạng thái
        self.drawing = False
        self.erasing = False
        self.last_saved = False
        self.map_counter = 1  # Đếm số map đã tạo
        self.last_filename = ""  # Tên file vừa save

    def handle_mouse_click(self, pos, button):
        """Xử lý click chuột"""
        x, y = pos

        # Kiểm tra nếu click trong vùng map
        if x < WINDOW_WIDTH and y < WINDOW_HEIGHT - 50:
            col = x // CELL_SIZE
            row = y // CELL_SIZE

            if 0 <= col < COLS and 0 <= row < ROWS:
                if button == 1:  # Left click - vẽ obstacle
                    self.grid[row][col] = 1
                    self.drawing = True
                elif button == 3:  # Right click - xóa obstacle
                    self.grid[row][col] = 0
                    self.erasing = True

    def handle_mouse_drag(self, pos):
        """Xử lý kéo chuột"""
        x, y = pos

        if x < WINDOW_WIDTH and y < WINDOW_HEIGHT - 50:
            col = x // CELL_SIZE
            row = y // CELL_SIZE

            if 0 <= col < COLS and 0 <= row < ROWS:
                if self.drawing:
                    self.grid[row][col] = 1
                elif self.erasing:
                    self.grid[row][col] = 0

    def save_map(self, filename=None):
        """Lưu map ra file txt với tên tự động"""
        if filename is None:
            # Tạo tên file tự động với timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"map_{self.map_counter:03d}_{timestamp}.txt"

        # Kiểm tra file đã tồn tại chưa, nếu có thì tăng counter
        while os.path.exists(filename):
            self.map_counter += 1
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"map_{self.map_counter:03d}_{timestamp}.txt"

        try:
            with open(filename, 'w') as f:
                # Ghi header (cols rows)
                f.write(f"{COLS} {ROWS}\n")

                # Ghi từng hàng
                for row in self.grid:
                    f.write(" ".join(map(str, row)) + "\n")

            print(f"Map saved successfully to {filename}")
            self.last_saved = True
            self.last_filename = filename  # Lưu tên file vừa save
            self.map_counter += 1  # Tăng counter cho lần save tiếp theo
            return True
        except Exception as e:
            print(f"Error saving map: {e}")
            return False

    def load_sample_map(self):
        """Tạo một số obstacle mẫu"""
        # Tạo một số vật cản mẫu
        # Tường biên
        for i in range(COLS):
            self.grid[0][i] = 1  # Tường trên
            self.grid[ROWS - 1][i] = 1  # Tường dưới

        for i in range(ROWS):
            self.grid[i][0] = 1  # Tường trái
            self.grid[i][COLS - 1] = 1  # Tường phải

        # Một số obstacle trong map
        for i in range(20, 40):
            for j in range(20, 25):
                self.grid[i][j] = 1

        for i in range(40, 50):
            for j in range(60, 80):
                self.grid[i][j] = 1

    def clear_map(self):
        """Xóa toàn bộ map"""
        self.grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]

    def draw_grid(self):
        """Vẽ lưới map"""
        for row in range(ROWS):
            for col in range(COLS):
                x = col * CELL_SIZE
                y = row * CELL_SIZE

                # Vẽ ô
                if self.grid[row][col] == 1:  # Obstacle
                    pygame.draw.rect(self.screen, BLACK, (x, y, CELL_SIZE, CELL_SIZE))
                else:  # Free space
                    pygame.draw.rect(self.screen, WHITE, (x, y, CELL_SIZE, CELL_SIZE))

                # Vẽ đường viền (chỉ vẽ với cell size lớn)
                if CELL_SIZE > 5:
                    pygame.draw.rect(self.screen, GRAY, (x, y, CELL_SIZE, CELL_SIZE), 1)

    def draw_info(self):
        """Vẽ thông tin hướng dẫn"""
        info_y = WINDOW_HEIGHT - 45

        instructions = [
            "Left Click: Add Obstacle | Right Click: Remove | SPACE: Save Map | C: Clear | L: Load Sample | ESC: Exit"
        ]

        for i, text in enumerate(instructions):
            color = GREEN if self.last_saved else RED
            text_surface = self.font.render(text, True, color)
            self.screen.blit(text_surface, (5, info_y + i * 20))

        # Đếm số obstacles
        obstacle_count = sum(sum(row) for row in self.grid)
        free_count = COLS * ROWS - obstacle_count

        # Hiển thị thông tin và file vừa save
        if self.last_saved and hasattr(self, 'last_filename'):
            stats_text = f"Obstacles: {obstacle_count} | Free: {free_count} | Saved: {self.last_filename}"
        else:
            stats_text = f"Obstacles: {obstacle_count} | Free: {free_count} | Total: {COLS}x{ROWS}"

        stats_surface = self.font.render(stats_text, True, RED)
        self.screen.blit(stats_surface, (5, info_y + 20))

    def run(self):
        """Chạy editor"""
        clock = pygame.time.Clock()
        running = True

        print("Map Editor Started!")
        print("Controls:")
        print("- Left Click/Drag: Add obstacles")
        print("- Right Click/Drag: Remove obstacles")
        print("- SPACE: Save map")
        print("- C: Clear map")
        print("- L: Load sample map")
        print("- ESC: Exit")

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        self.save_map()
                    elif event.key == pygame.K_c:
                        self.clear_map()
                        self.last_saved = False
                    elif event.key == pygame.K_l:
                        self.load_sample_map()
                        self.last_saved = False

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self.handle_mouse_click(event.pos, event.button)
                    self.last_saved = False

                elif event.type == pygame.MOUSEBUTTONUP:
                    self.drawing = False
                    self.erasing = False

                elif event.type == pygame.MOUSEMOTION:
                    if pygame.mouse.get_pressed()[0] or pygame.mouse.get_pressed()[2]:
                        self.handle_mouse_drag(event.pos)
                        self.last_saved = False

            # Vẽ màn hình
            self.screen.fill(WHITE)
            self.draw_grid()
            self.draw_info()

            pygame.display.flip()
            clock.tick(60)

        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    editor = MapEditor()
    editor.run()