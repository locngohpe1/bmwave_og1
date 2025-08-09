import math
import numpy as np
import pygame as pg
import time
import csv
import random

from a_star import GridMapGraph, a_star_search
from logic import Logic, Q
from grid_map import Grid_Map

# coverage:             1 unit of energy / cell width
# advance & retreat:    0.5 unit of energy / cell width
ENERGY_CAPACITY = 1000

ui = Grid_Map()
ui.read_map('map/experiment/scenario1/map_1.txt')
ENVIRONMENT, battery_pos = ui.edit_map()

# Setup callback for unknown obstacles
def on_unknown_obstacle_detected(pos):
    """Callback when user creates unknown obstacle"""
    if 'robot' in globals():
        robot.add_manual_obstacle(pos)

ui.set_robot_callback(on_unknown_obstacle_detected)

ROW_COUNT = len(ENVIRONMENT)
COL_COUNT = len(ENVIRONMENT[0])

FPS = 80

total_travel_length = 0
coverage_length, retreat_length, advance_length = 0, 0, 0
return_charge_count = 1
count_cell_go_through = 1
deadlock_count = 0
extreme_deadlock_count = 0
execute_time = time.time()

# Find special area
from optimization import get_special_area
special_areas = get_special_area(ENVIRONMENT)

# Pre-calculate return path to charge station from each cell in ENVIRONMENT
from optimization import return_path_matrix, get_return_path
return_matrix = return_path_matrix(ENVIRONMENT, battery_pos)

def check_valid_pos(pos):
    row, col = pos
    if row < 0 or row >= ROW_COUNT: return False
    if col < 0 or col >= COL_COUNT: return False
    return True

class Robot:
    def __init__(self, battery_pos, map_row_count, map_col_count):
        self.logic = Logic(map_row_count, map_col_count)
        '''
        map:
            'u': unvisited
            'e': explored
            'o': obstacle (known)
            'n': new_obstacle (unknown, detected)
        '''
        self.map = None
        self.current_pos = battery_pos

        # The angle between the robot direction and left to right axis in rad [0, 2pi)
        self.angle = math.pi / 2

        self.battery_pos = battery_pos
        self.energy = ENERGY_CAPACITY
        self.estimated_return_energy = 0

        self.move_status = 0  # 0: normal coverage, 1: retreat, 2: charge, 3: advance
        self.cache_path = []  # store temporary path (e.g.: retreat, advance)

        # Simple obstacle tracking - no neural networks needed
        self.known_environment = None
        self.dynamic_obstacles = set()  # Track unknown obstacles
        self.obstacle_avoidance_count = 0  # Statistics

    def set_map(self, environment):
        row_count, col_count = len(environment), len(environment[0])
        self.map = np.full((row_count, col_count), 'u')
        self.known_environment = environment.copy()  # Store original known environment

        for x in range(len(environment)):
            for y in range(len(environment[0])):
                if environment[x, y] == 1:
                    self.map[x, y] = 'o'

        self.logic.set_weight_map(environment)

    def add_manual_obstacle(self, pos):
        """Handle manually added unknown obstacles via Shift+Click"""
        x, y = pos
        if (0 <= x < len(self.map) and 0 <= y < len(self.map[0]) and
                self.known_environment[x, y] == 0):  # Was free in known environment

            self.map[x, y] = 'n'  # Mark as new obstacle
            self.dynamic_obstacles.add((x, y))
            self.logic.weight_map[x, y] = -1  # Update BWave weight map
            print(f"Manual obstacle added at {pos}")

    def detect_new_obstacles(self):
        """Simple sensor simulation - detect new obstacles in 8-connected neighbors"""
        current_pos = self.current_pos
        new_obstacles = []

        # Check 8-connected neighbors
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                x, y = current_pos[0] + dx, current_pos[1] + dy

                # Check bounds
                if (x < 0 or x >= len(self.map) or y < 0 or y >= len(self.map[0])):
                    continue

                # 2% probability of discovering new obstacle per neighbor
                if (self.known_environment[x, y] == 0 and  # Was free in known environment
                        self.map[x, y] == 'u' and  # Currently unvisited
                        random.random() < 0.02):  # 2% discovery chance

                    self.map[x, y] = 'n'  # Mark as new obstacle
                    self.dynamic_obstacles.add((x, y))
                    new_obstacles.append((x, y))

                    # Update BWave weight map
                    self.logic.weight_map[x, y] = -1
                    print(f"Sensor detected obstacle at {(x, y)}")

        return new_obstacles

    def avoid_new_obstacle(self, blocked_cell, available_wp):
        """Simple obstacle avoidance using BWave waypoints"""
        current_pos = self.current_pos

        # Priority 1: Choose safe alternative from BWave's available waypoints
        safe_wp = [wp for wp in available_wp if wp not in self.dynamic_obstacles]
        if safe_wp:
            self.obstacle_avoidance_count += 1
            return min(safe_wp, key=self.travel_cost)  # Use BWave's selection logic

        # Priority 2: Try 4-directional neighbors
        neighbors = [
            (current_pos[0] - 1, current_pos[1]),  # Up
            (current_pos[0] + 1, current_pos[1]),  # Down
            (current_pos[0], current_pos[1] - 1),  # Left
            (current_pos[0], current_pos[1] + 1)   # Right
        ]

        for neighbor in neighbors:
            if (0 <= neighbor[0] < len(self.map) and
                    0 <= neighbor[1] < len(self.map[0]) and
                    self.map[neighbor] not in ('o', 'n') and
                    neighbor not in self.dynamic_obstacles):
                self.obstacle_avoidance_count += 1
                return neighbor

        # Last resort: stay in place (BWave deadlock handling will take over)
        return current_pos

    def run(self):
        global FPS, deadlock_count, extreme_deadlock_count
        clock = pg.time.Clock()
        run = True
        pause = False
        coverage_finish = False

        while run:
            ui.draw()
            clock.tick(FPS)
            for event in pg.event.get():
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_SPACE:  # pause
                        pause = not pause
                        pg.image.save(ui.WIN, 'tmp/screenshot.png')
                    elif event.key == pg.K_LEFT:  # slow down
                        FPS /= 2
                    elif event.key == pg.K_RIGHT:  # speed up
                        FPS *= 2
                if event.type == pg.QUIT:
                    run = False

            if pause:
                continue

            if self.logic.state == Q.FINISH:
                if not coverage_finish:
                    coverage_finish = True
                    self.retreat()
                    self.charge()

                    global execute_time
                    execute_time = time.time() - execute_time
                    print('Coverage Finish')

                continue

            # Sensor detection phase
            new_obstacles = self.detect_new_obstacles()
            if new_obstacles:
                print(f"New obstacles detected: {new_obstacles}")

            # BWave decision making with simple obstacle avoidance
            wp = self.logic.get_wp(self.current_pos)
            if len(wp) == 0:
                continue

            # BWave selects best waypoint
            selected_cell = self.select_from_wp(wp)

            # Simple check: if BWave's choice hits unknown obstacle, find alternative
            if selected_cell in self.dynamic_obstacles:
                selected_cell = self.avoid_new_obstacle(selected_cell, wp)
                print(f"Obstacle avoidance: redirected to {selected_cell}")

            if selected_cell == self.current_pos:
                self.task()
            else:
                # BWave normal operation
                if self.logic.state == Q.NORMAL:
                    if self.check_enough_energy(selected_cell) == False:
                        self.charge_planning()
                        continue
                    self.move_to(selected_cell)

                # BWave deadlock handling
                elif self.logic.state == Q.DEADLOCK:
                    path, dist = self.logic.cache_path, self.logic.cache_dist
                    print(f"BWave deadlock escape ({round(dist, 2)})")

                    deadlock_count += 1
                    if dist > math.sqrt(ROW_COUNT ** 2 + COL_COUNT ** 2) / 4:
                        extreme_deadlock_count += 1

                    self.follow_path_plan(path, time_delay=0.05, check_energy=True, stop_on_unexpored=True)

    def select_from_wp(self, wp):
        new_wp = self.get_better_wp(wp)
        if len(new_wp) > 0: wp = new_wp
        return min(wp, key=self.travel_cost)

    def task(self):
        current_pos = self.current_pos
        self.map[current_pos] = 'e'
        self.logic.update_explored(current_pos)
        ui.task(current_pos)

    def move_to(self, pos):
        global total_travel_length, coverage_length, retreat_length, advance_length, count_cell_go_through

        dist = energy = math.dist(self.current_pos, pos)

        if self.move_status in (1, 3):  # retreat or advance cost half energy as coverage
            energy = 0.5 * energy

        if self.energy < energy:
            raise Exception('Robot run out of battery')
        self.energy -= energy

        self.rotate_to(pos)
        self.current_pos = pos

        if self.move_status == 0:
            ui.move_to(pos)
            coverage_length += dist
        elif self.move_status == 1:
            ui.move_retreat(pos)
            retreat_length += dist
        elif self.move_status == 3:
            ui.move_advance(pos)
            advance_length += dist

        total_travel_length += dist
        if self.move_status == 0:  # coverage
            count_cell_go_through += 1

        ui.set_energy_display(self.energy)

    def travel_cost(self, pos_to):
        pos_from = self.current_pos
        turn_angle = abs(self.angle - self.get_angle(pos_to))
        if turn_angle > math.pi:  # always take the smaller angle to turn
            turn_angle = 2 * math.pi - turn_angle
        travel_dist = math.dist(pos_from, pos_to)

        # BWave's cost function: travel distance + turning cost
        cost = 2 * travel_dist + 1 * turn_angle
        return cost

    def get_angle(self, pos_to):
        pos_from = self.current_pos
        vecto = (pos_to[0] - pos_from[0], pos_to[1] - pos_from[1])
        angle = - np.arctan2(vecto[0], vecto[1])
        return angle % (2 * math.pi)

    def rotate_to(self, pos_to):
        self.angle = self.get_angle(pos_to)

    def check_enough_energy(self, wp):
        return_dist_from_wp = return_matrix[wp][1]
        expected_energy = math.dist(self.current_pos, wp) + 0.5 * return_dist_from_wp
        if self.energy < expected_energy:
            return False
        else:
            return True

    def charge_planning(self):
        global return_charge_count
        return_charge_count += 1
        self.retreat()
        self.charge()
        time.sleep(0.1)
        self.advance()
        self.move_status = 0

    def retreat(self):
        return_path = get_return_path(return_matrix, self.current_pos)
        self.cache_path = return_path
        self.move_status = 1
        ui.set_charge_path(return_path)
        self.follow_path_plan(return_path, time_delay=0.05)

    def charge(self):
        self.move_status = 2
        self.energy = ENERGY_CAPACITY

    def advance(self):
        self.move_status = 3
        advance_path = list(reversed(self.cache_path))
        ui.set_charge_path(advance_path)
        self.follow_path_plan(advance_path, time_delay=0.05)

    def follow_path_plan(self, path, time_delay=0, check_energy=False, stop_on_unexpored=False):
        clock = pg.time.Clock()
        for pos in path:
            clock.tick(FPS / 4)

            while check_energy == True and self.check_enough_energy(pos) == False:
                self.charge_planning()

            self.move_to(pos)
            ui.draw()

            if stop_on_unexpored:
                if self.logic.weight_map[pos] > 0: return

    def get_better_wp(self, wp):
        if len(wp) == 1: return wp

        new_wp = []
        x_up, y_up = min(wp, key=lambda x: x[0])
        x_down, y_down = max(wp, key=lambda x: x[0])

        if not check_valid_pos((x_up - 1, y_up)) or self.map[(x_up - 1, y_up)] in ('o', 'e'):
            new_wp.append((x_up, y_up))
        if not check_valid_pos((x_down + 1, y_down)) or self.map[(x_down + 1, y_down)] in ('o', 'e'):
            new_wp.append((x_down, y_down))
        return new_wp

    def set_special_areas(self, special_areas):
        self.logic.set_special_areas(special_areas)
        self.set_inner_special_areas(special_areas)

    def set_inner_special_areas(self, special_areas):
        candidate_areas = get_special_area(ENVIRONMENT, reverse_dir=True)
        for parent_region in special_areas:
            for child_region in candidate_areas:
                if not (set(child_region.cell_list) <= set(parent_region.cell_list)): continue

                floor_weight = -1
                for pos in parent_region.cell_list:
                    if pos[1] == child_region.max_y:
                        floor_weight = self.logic.weight_map[pos] + 2
                        break

                for x, y in child_region.cell_list:
                    self.logic.weight_map[x, y] = floor_weight + (child_region.max_y - y)

def main():
    global robot
    robot = Robot(battery_pos, ROW_COUNT, COL_COUNT)
    robot.set_map(ENVIRONMENT)
    robot.set_special_areas(special_areas)

    global execute_time
    execute_time = time.time()
    robot.run()

    print('\nCoverage:\t', coverage_length)
    print('Retreat:\t', retreat_length)
    print('Advance:\t', advance_length)
    print('-' * 8)
    print('Total:', total_travel_length)

    overlap_rate = (count_cell_go_through / np.sum(robot.map == 'e') - 1) * 100
    print('\nOverlap rate: ', overlap_rate)
    print('Number Of Return: ', return_charge_count)
    print('Number of extreme deadlock:', extreme_deadlock_count, '/', deadlock_count)
    print('Time: ', execute_time)

    # BWave + Obstacle Avoidance Statistics
    if len(robot.dynamic_obstacles) > 0:
        print(f'Unknown obstacles detected: {len(robot.dynamic_obstacles)}')
        print(f'Obstacle avoidance activations: {robot.obstacle_avoidance_count}')
        print(f'Obstacle positions: {list(robot.dynamic_obstacles)}')
    else:
        print('No unknown obstacles encountered - pure BWave performance')

    # Coverage efficiency
    total_free_cells = np.sum(robot.known_environment == 0)
    covered_cells = np.sum(robot.map == 'e')
    coverage_percentage = (covered_cells / total_free_cells) * 100
    print(f'Coverage efficiency: {coverage_percentage:.2f}%')

if __name__ == "__main__":
    main()