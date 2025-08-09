import numpy as np
import pygame as pg
import copy
import colorsys
import random

# Color
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREY = (197, 198, 208)
RED = (255, 0, 0)
GREEN = (108, 217, 115)
YELLOW = (255, 243, 128)
BLUE = (5, 16, 148)
LIGHT_BLUE = (0, 191, 255)
BROWN = (76, 1, 33)
LIGHT_ORANGE = (255, 140, 0)

# epsilon value (size of cell)
EPSILON = 8

BORDER = 1
INFO_BAR_HEIGHT = 30
GAP = 3

pg.init()
font = pg.font.SysFont(None, 30)


def hsv2rgb(h, s, v):
    (r, g, b) = colorsys.hsv_to_rgb(h, s, v)
    return (int(255 * r), int(255 * g), int(255 * b))


def getDistinctColors(n):
    huePartition = 1.0 / (n + 1)
    # return [hsv2rgb(huePartition * value, 1.0, 1.0) for value in range(0, n)]
    return [LIGHT_BLUE] * n


class Grid_Map:
    def __init__(self):
        # pg.init()
        pg.display.set_caption("ε⋆+ Coverage Path Planning")
        self.WIN = None
        self.grid_surface = None

        # ✅ FIXED: Initialize map as None, will be set properly later
        self.map = None
        self.row_count = 0
        self.col_count = 0

        # ✅ FIXED: Battery position will be set by user interaction
        self.battery_pos = (0, 0)
        self.vehicle_pos = (0, 0)

        self.battery_img = pg.Rect(BORDER, BORDER, EPSILON - BORDER, EPSILON - BORDER)

        self.trajectories = [[(0, 0)]]  # list of trajectories (currently only coverage path)

        self.move_status = 0  # 0: normal coverage, 1: retreat, 2: charge, 3: advance
        self.charge_path_plan = []  # share between retreat & advance

        self.info_bar = None
        self.energy_display = 0

    def read_map(self, filepath):
        """Read map file và return environment + default battery position"""
        with open(filepath, "r", encoding="utf-8") as f:
            self.col_count, self.row_count = [int(i) for i in f.readline().strip().split()]

            display_size = [EPSILON * self.col_count + 1, EPSILON * self.row_count + INFO_BAR_HEIGHT]
            self.info_bar = pg.Rect(0, EPSILON * self.row_count + BORDER, EPSILON * self.col_count,
                                    EPSILON * self.row_count + INFO_BAR_HEIGHT)
            self.WIN = pg.display.set_mode(display_size)
            self.grid_surface = pg.Surface((EPSILON * self.col_count + 1, EPSILON * self.row_count))

            map_list = []
            for idx, line in enumerate(f):
                line = [int(value) for value in line.strip().split()]
                line = line[0:self.col_count]
                map_list.append(line)

            if len(map_list) == 0:
                map_list = [[0 for _ in range(self.col_count)] for _ in range(self.row_count)]

            # ✅ FIXED: Always convert to numpy array immediately
            self.map = np.array(map_list, dtype=object)

        # ✅ NOTE: This returns DEFAULT battery_pos, will be updated in edit_map()
        return copy.deepcopy(map_list), self.battery_pos

    def edit_map(self):
        """
        🔧 FIXED: Interactive map editing với proper battery position return
        Left click: Place/remove obstacles
        Right click: Set charging station
        Any key: Start algorithm

        Returns: (final_environment, final_battery_position)
        """
        print("\n🎮 INTERACTIVE SETUP MODE:")
        print("• Left click: Place/remove obstacles")
        print("• Right click: Set charging station (IMPORTANT!)")
        print("• Press any key when ready to start algorithm")
        print("\n⚠️  MUST set charging station before starting!")

        # ✅ FIXED: Initialize with proper numpy array if not exists
        if self.map is None:
            self.map = np.zeros((self.row_count, self.col_count), dtype=object)

        done = False
        draw_obstacle = False
        prev_cell = None
        battery_set = False  # Track if battery position was set

        while done == False:
            pos = pg.mouse.get_pos()
            col = pos[0] // EPSILON
            row = pos[1] // EPSILON

            for event in pg.event.get():
                if event.type == pg.QUIT:
                    done = True
                elif event.type == pg.KEYDOWN:
                    if not battery_set:
                        print("⚠️  WARNING: No charging station set! Using default (0,0)")
                        print("   Please set charging station with RIGHT CLICK first!")
                        continue
                    done = True  # ✅ Start algorithm on any keypress
                elif event.type == pg.MOUSEBUTTONDOWN:
                    mouse_pressed = pg.mouse.get_pressed()
                    if mouse_pressed[0]:  # left mouse click: obstacle
                        draw_obstacle = True
                    if mouse_pressed[2]:  # right mouse click: charging station
                        if self.check_valid_pos((row, col)) == False:
                            continue

                        # ✅ FIXED: Update battery position properly
                        self.update_battery_pos((row, col))
                        self.trajectories[0] = [(row, col)]

                        # Clear any obstacle at this position
                        if isinstance(self.map, np.ndarray):
                            self.map[row, col] = 0
                        else:
                            self.map[row][col] = 0

                        battery_set = True
                        print(f"🔋 Charging station set at: ({row}, {col})")

                elif event.type == pg.MOUSEBUTTONUP:
                    draw_obstacle = False
                    prev_cell = None

            # check boolean flag to allow holding left click to draw
            if draw_obstacle:
                if self.check_valid_pos((row, col)) == False:
                    continue
                if (prev_cell != (row, col)):
                    prev_cell = (row, col)
                    current_val = self.map[row, col] if isinstance(self.map, np.ndarray) else self.map[row][col]
                    if current_val == 0 and self.battery_pos != (row, col):
                        if isinstance(self.map, np.ndarray):
                            self.map[row, col] = 1
                        else:
                            self.map[row][col] = 1
                    else:
                        if isinstance(self.map, np.ndarray):
                            self.map[row, col] = 0
                        else:
                            self.map[row][col] = 0

            # pygame draw
            self.draw_map()
            pg.draw.rect(self.WIN, YELLOW, self.battery_img)

            # Show status
            if battery_set:
                status_text = f"Battery: {self.battery_pos} - Press any key to start"
                color = GREEN
            else:
                status_text = "Right-click to set charging station"
                color = RED

            text_surface = font.render(status_text, True, color)
            self.WIN.blit(text_surface, (10, self.info_bar.y + 5))

            pg.display.flip()

        # ✅ FIXED: Always ensure numpy array at the end
        if not isinstance(self.map, np.ndarray):
            self.map = np.array(self.map, dtype=object)

        print(f"✅ Setup complete! Charging station: {self.battery_pos}")

        # ✅ FIXED: Return BOTH environment AND updated battery position
        return copy.deepcopy(self.map.tolist()), self.battery_pos

    def save_map(self, output_file):
        map = self.map

        with open(output_file, "w", encoding="utf-8") as f:
            col_count, row_count = len(map[0]), len(map)
            f.write(str(col_count) + ' ' + str(row_count) + '\n')
            for row in map:
                line = [str(value) for value in row]
                line = " ".join(line)
                f.write(line + '\n')
            print("Save map done!")

    def draw(self):
        # self.draw_map()
        self.WIN.blit(self.grid_surface, (0, 0))
        pg.draw.rect(self.WIN, (238, 238, 238), self.info_bar)

        for coverage_path in self.trajectories:
            self.draw_path(coverage_path)

        if self.move_status == 1:  # retreat
            self.draw_path(self.charge_path_plan, BROWN)
        elif self.move_status == 3:  # advance
            self.draw_path(self.charge_path_plan, BLUE)

        pg.draw.rect(self.WIN, YELLOW, self.battery_img)

        vehicle_center = ((self.vehicle_pos[1] + 1 / 2) * EPSILON + BORDER,
                          (self.vehicle_pos[0] + 1 / 2) * EPSILON + BORDER)
        pg.draw.circle(self.WIN, BLACK, vehicle_center, EPSILON / 3, width=0)

        energy_display_img = font.render('Energy: ' + str(self.energy_display), True, RED)
        self.WIN.blit(energy_display_img, (self.info_bar.x + 5, self.info_bar.y + 5))

        pg.display.flip()

    def draw_map(self):
        self.grid_surface.fill(BLACK)
        for row in range(len(self.map)):
            for col in range(len(self.map[0]) if len(self.map) > 0 else 0):
                color = WHITE

                # ✅ FIXED: Handle both list and numpy array indexing
                if isinstance(self.map, np.ndarray):
                    cell_value = self.map[row, col]
                else:
                    cell_value = self.map[row][col]

                if cell_value in (1, 'o'):  # dùng chung cho map.txt
                    color = BLACK
                elif cell_value == '_':
                    color = GREY
                elif cell_value == 'e':
                    color = GREEN

                pg.draw.rect(self.grid_surface,
                             color,
                             [EPSILON * col + BORDER,
                              EPSILON * row + BORDER,
                              EPSILON - BORDER,
                              EPSILON - BORDER])

        self.WIN.blit(self.grid_surface, (0, 0))
        pg.draw.rect(self.WIN, (238, 238, 238), self.info_bar)

    def illustrate_regions(self, decomposed, region_count):
        self.WIN.fill(BLACK)
        pg.draw.rect(self.WIN, (238, 238, 238), self.info_bar)
        region_colors = getDistinctColors(region_count)
        random.shuffle(region_colors)

        for row in range(len(decomposed)):
            for col in range(len(decomposed[0])):
                color = BLACK
                region = decomposed[row][col]

                if region == -1:
                    color = WHITE
                elif region != 0:
                    color = region_colors[region - 1]

                pg.draw.rect(self.WIN,
                             color,
                             [EPSILON * col + BORDER,
                              EPSILON * row + BORDER,
                              EPSILON - BORDER,
                              EPSILON - BORDER])

        pg.display.flip()

    def illustrate_inner_special_regions(self, special_areas, inner_special_areas):
        self.WIN.fill(BLACK)
        pg.draw.rect(self.WIN, (238, 238, 238), self.info_bar)

        for row in range(len(self.map)):
            for col in range(len(self.map[0]) if len(self.map) > 0 else 0):
                color = BLACK

                # ✅ FIXED: Handle both list and numpy array indexing
                if isinstance(self.map, np.ndarray):
                    cell_value = self.map[row, col]
                else:
                    cell_value = self.map[row][col]

                if cell_value == 0:
                    color = WHITE

                pg.draw.rect(self.WIN,
                             color,
                             [EPSILON * col + BORDER,
                              EPSILON * row + BORDER,
                              EPSILON - BORDER,
                              EPSILON - BORDER])

        for region in special_areas:
            for (row, col) in region.cell_list:
                pg.draw.rect(self.WIN,
                             LIGHT_BLUE,
                             [EPSILON * col + BORDER,
                              EPSILON * row + BORDER,
                              EPSILON - BORDER,
                              EPSILON - BORDER])

        for region in inner_special_areas:
            for (row, col) in region.cell_list:
                pg.draw.rect(self.WIN,
                             LIGHT_ORANGE,
                             [EPSILON * col + BORDER,
                              EPSILON * row + BORDER,
                              EPSILON - BORDER,
                              EPSILON - BORDER])

        pg.display.flip()

    def draw_path(self, path, color=RED, width=2):
        point_list = [(EPSILON * pos[1] + EPSILON / 2, EPSILON * pos[0] + EPSILON / 2) for pos in path]
        if len(point_list) > 1:
            pg.draw.lines(self.WIN, color, False, point_list, width=2)

    def update_battery_pos(self, pos):
        """✅ FIXED: Update battery position and UI"""
        self.battery_pos = pos
        self.battery_img.x = EPSILON * pos[1] + BORDER
        self.battery_img.y = EPSILON * pos[0] + BORDER
        print(f"🔋 Battery position updated to: {pos}")

    def update_vehicle_pos(self, pos):
        self.vehicle_pos = pos

    # ✅ FIXED: Handle both list and numpy array cases (BWave compatible)
    def task(self, pos):
        """
        Mark cell as explored ('e')
        pos: tuple (row, col)
        """
        row, col = pos  # Unpack tuple

        # Handle both list and numpy array cases
        if isinstance(self.map, np.ndarray):
            self.map[row, col] = 'e'  # Numpy array indexing
        else:
            self.map[row][col] = 'e'  # List indexing

        color = GREEN
        pg.draw.rect(self.grid_surface,
                     color,
                     [EPSILON * col + BORDER,
                      EPSILON * row + BORDER,
                      EPSILON - BORDER,
                      EPSILON - BORDER])

    # ✅ FIXED: Handle tuple position properly
    def move_to(self, pos):
        """
        Move to position during coverage
        pos: tuple (row, col)
        """
        if self.move_status != 0:
            self.trajectories[-1].append(self.vehicle_pos)
        self.move_status = 0
        self.update_vehicle_pos(pos)
        self.trajectories[-1].append(pos)

    # ✅ FIXED: Handle tuple position properly
    def move_retreat(self, pos):
        """
        Move to position during retreat
        pos: tuple (row, col)
        """
        self.move_status = 1
        self.update_vehicle_pos(pos)
        if pos == self.battery_pos:
            self.move_status = 2

    # ✅ FIXED: Handle tuple position properly
    def move_advance(self, pos):
        """
        Move to position during advance
        pos: tuple (row, col)
        """
        if self.move_status != 3:
            self.trajectories.append([])
        self.move_status = 3
        self.update_vehicle_pos(pos)

    def set_charge_path(self, path):
        self.charge_path_plan = path

    def set_energy_display(self, energy):
        self.energy_display = round(energy, 2)

    def check_valid_pos(self, pos):
        row, col = pos
        if row < 0 or row >= self.row_count: return False
        if col < 0 or col >= self.col_count: return False
        return True