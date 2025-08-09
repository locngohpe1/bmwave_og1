import math
import numpy as np
import pygame as pg
import time
import csv
import torch
import torch.nn as nn
import torch.optim as optim
import random
from collections import deque

from a_star import GridMapGraph, a_star_search
from logic import Logic, Q
from grid_map import Grid_Map

# coverage:             1 unit of energy / cell width
# advance & retreat:    0.5 unit of energy / cell width
ENERGY_CAPACITY = 1000


# Q-Learning Neural Network
class QNetwork(nn.Module):
    def __init__(self, state_channels=5, grid_size=16, action_size=4):
        super(QNetwork, self).__init__()
        # Convolutional layers for spatial feature extraction
        self.conv1 = nn.Conv2d(state_channels, 16, kernel_size=5, padding=2)
        self.conv2 = nn.Conv2d(16, 16, kernel_size=5, padding=2)
        self.bn1 = nn.BatchNorm2d(16)
        self.bn2 = nn.BatchNorm2d(16)

        # Calculate conv output size
        conv_output_size = 16 * grid_size * grid_size

        # LSTM for memory
        self.lstm = nn.LSTM(conv_output_size + 1, 128, batch_first=True)  # +1 for remaining energy

        # Output layer
        self.fc = nn.Linear(128, action_size)

    def forward(self, state, energy, hidden=None):
        batch_size = state.size(0)

        # Convolutional layers
        x = torch.relu(self.bn1(self.conv1(state)))
        x = torch.relu(self.bn2(self.conv2(x)))

        # Flatten conv output
        x = x.view(batch_size, -1)

        # Concatenate with energy
        energy = energy.view(batch_size, 1)
        x = torch.cat([x, energy], dim=1)

        # LSTM
        x = x.unsqueeze(1)  # Add sequence dimension
        lstm_out, hidden = self.lstm(x, hidden)

        # Output Q-values
        q_values = self.fc(lstm_out.squeeze(1))

        return q_values, hidden


ui = Grid_Map()
ui.read_map('map/experiment/scenario1/map_1.txt')
ENVIRONMENT, battery_pos = ui.edit_map()

# Setup callback for unknown obstacles
def on_unknown_obstacle_detected(pos):
    """Callback when user creates unknown obstacle"""
    if 'robot' in globals():
        robot.add_manual_obstacle(pos)

ui.set_robot_callback(on_unknown_obstacle_detected)
# ui.save_map('map/empty_map.txt')

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
            'o': obstacle
            'n': new_obstacle (detected by Q-learning)
        '''
        self.map = None
        self.current_pos = battery_pos

        # The angle between the robot direction and left to right axis in rad [0, 2pi)
        # (up direction at the start)
        self.angle = math.pi / 2

        self.battery_pos = battery_pos
        self.energy = ENERGY_CAPACITY
        self.estimated_return_energy = 0

        self.move_status = 0  # 0: normal coverage, 1: retreat, 2: charge, 3: advance
        self.cache_path = []  # store temporary path (e.g.: retreat, advance)

        # Simplified - no Q-Learning neural network needed
        # Just track dynamic obstacles for avoidance
        self.q_network = QNetwork(state_channels=5, grid_size=max(map_row_count, map_col_count))
        self.target_network = QNetwork(state_channels=5, grid_size=max(map_row_count, map_col_count))
        self.target_network.load_state_dict(self.q_network.state_dict())
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=0.001)
        self.replay_buffer = deque(maxlen=10000)
        self.epsilon = 0.3  # exploration rate
        self.gamma = 0.9  # discount factor
        self.hidden_state = None
        self.update_counter = 0

        # Hybrid control flags
        self.new_obstacles_detected = False
        self.known_environment = None
        self.dynamic_obstacles = set()
        self.last_state = None
        self.last_energy = None
        self.last_action = None

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
        """Handle manually added unknown obstacles"""
        x, y = pos
        if (0 <= x < len(self.map) and 0 <= y < len(self.map[0]) and
                self.known_environment[x, y] == 0):  # Was free in known environment

            self.map[x, y] = 'n'  # Mark as new obstacle
            self.dynamic_obstacles.add((x, y))
            self.new_obstacles_detected = True
            self.logic.weight_map[x, y] = -1  # Update weight map
            print(f"Manual obstacle added at {pos}")

    def detect_new_obstacles(self):
        """Detect new obstacles and update BWave weight map"""
        current_pos = self.current_pos
        new_obstacles = []

        # Check 8-connected neighbors for new obstacles
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                x, y = current_pos[0] + dx, current_pos[1] + dy

                # Check bounds
                if (x < 0 or x >= len(self.map) or y < 0 or y >= len(self.map[0])):
                    continue

                # Simulate 3% chance of discovering new obstacle (reduced)
                if (self.known_environment[x, y] == 0 and  # Was free in known environment
                        self.map[x, y] == 'u' and  # Currently unvisited
                        random.random() < 0.03):  # 3% discovery chance

                    self.map[x, y] = 'n'  # Mark as new obstacle
                    self.dynamic_obstacles.add((x, y))
                    new_obstacles.append((x, y))

                    # Update BWave weight map to mark as obstacle
                    self.logic.weight_map[x, y] = -1
                    print(f"New obstacle detected at {(x, y)}")

        return new_obstacles

    def get_q_learning_state(self):
        """Create 5-channel state tensor for Q-learning"""
        row_count, col_count = len(self.map), len(self.map[0])

        # Use max dimension to create square tensor for CNN
        max_dim = max(row_count, col_count)
        state = np.zeros((5, max_dim, max_dim), dtype=np.float32)

        # Channel 0: Known obstacles
        for x in range(row_count):
            for y in range(col_count):
                if self.known_environment[x, y] == 1:
                    state[0, x, y] = 1.0

        # Channel 1: New obstacles
        for x in range(row_count):
            for y in range(col_count):
                if self.map[x, y] == 'n':
                    state[1, x, y] = 1.0

        # Channel 2: Current position
        if (self.current_pos[0] < max_dim and self.current_pos[1] < max_dim):
            state[2, self.current_pos[0], self.current_pos[1]] = 1.0

        # Channel 3: Visited cells
        for x in range(row_count):
            for y in range(col_count):
                if self.map[x, y] == 'e':
                    state[3, x, y] = 1.0

        # Channel 4: Charging station
        if (self.battery_pos[0] < max_dim and self.battery_pos[1] < max_dim):
            state[4, self.battery_pos[0], self.battery_pos[1]] = 1.0

        return torch.FloatTensor(state).unsqueeze(0)  # Add batch dimension


    def in_unknown_region(self):
        """Check if current area has new obstacles nearby"""
        current_pos = self.current_pos

        # Check 3x3 area around current position
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                x, y = current_pos[0] + dx, current_pos[1] + dy

                if (0 <= x < len(self.map) and 0 <= y < len(self.map[0]) and
                        (x, y) in self.dynamic_obstacles):
                    return True
        return False

    def avoid_new_obstacle(self, blocked_cell, available_wp):
        """Simple obstacle avoidance - choose alternative from BWave waypoints"""
        current_pos = self.current_pos

        # Filter out blocked waypoints
        safe_wp = []
        for wp in available_wp:
            if wp not in self.dynamic_obstacles:
                safe_wp.append(wp)

        if safe_wp:
            # Use BWave's own selection logic on safe waypoints
            return min(safe_wp, key=self.travel_cost)

        # If no safe waypoints, try 4-directional neighbors
        neighbors = [
            (current_pos[0] - 1, current_pos[1]),  # Up
            (current_pos[0] + 1, current_pos[1]),  # Down
            (current_pos[0], current_pos[1] - 1),  # Left
            (current_pos[0], current_pos[1] + 1)  # Right
        ]

        for neighbor in neighbors:
            if (0 <= neighbor[0] < len(self.map) and
                    0 <= neighbor[1] < len(self.map[0]) and
                    self.map[neighbor] not in ('o', 'n') and
                    neighbor not in self.dynamic_obstacles):
                return neighbor

        # Last resort: stay in place
        return current_pos

    def q_learning_action_selection(self):
        """Select action using Q-learning"""
        state = self.get_q_learning_state()
        energy = torch.FloatTensor([self.energy / ENERGY_CAPACITY]).unsqueeze(0)

        with torch.no_grad():
            q_values, self.hidden_state = self.q_network(state, energy, self.hidden_state)

        # Get valid actions only
        valid_actions = []
        current_pos = self.current_pos
        action_map = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

        for action_idx, (dx, dy) in enumerate(action_map):
            next_pos = (current_pos[0] + dx, current_pos[1] + dy)
            if (0 <= next_pos[0] < len(self.map) and
                    0 <= next_pos[1] < len(self.map[0]) and
                    self.map[next_pos] not in ('o', 'n')):  # Not obstacle
                valid_actions.append((action_idx, next_pos))

        if not valid_actions:
            return current_pos

        # Epsilon-greedy action selection from valid actions only
        if random.random() < self.epsilon:
            action_idx, next_pos = random.choice(valid_actions)
        else:
            # Choose best action from valid actions
            best_action_idx = -1
            best_q_value = float('-inf')
            best_next_pos = current_pos

            for action_idx, next_pos in valid_actions:
                if q_values[0][action_idx] > best_q_value:
                    best_q_value = q_values[0][action_idx]
                    best_action_idx = action_idx
                    best_next_pos = next_pos

            action_idx = best_action_idx
            next_pos = best_next_pos

        # Store action for experience replay
        self.last_action = action_idx

        return next_pos

    def learn_from_experience(self):
        """Train Q-network using experience replay"""
        if len(self.replay_buffer) < 100:  # Wait for enough experiences
            return

        # Sample batch from replay buffer
        batch = random.sample(self.replay_buffer, min(32, len(self.replay_buffer)))

        states = torch.stack([exp[0].squeeze(0) for exp in batch])
        energies = torch.stack([exp[1].squeeze(0) for exp in batch])
        actions = torch.tensor([exp[2] for exp in batch])
        rewards = torch.tensor([exp[3] for exp in batch], dtype=torch.float32)
        next_states = torch.stack([exp[4].squeeze(0) for exp in batch])
        next_energies = torch.stack([exp[5].squeeze(0) for exp in batch])
        dones = torch.tensor([exp[6] for exp in batch], dtype=torch.bool)

        # Current Q-values
        current_q_values, _ = self.q_network(states, energies)
        current_q_values = current_q_values.gather(1, actions.unsqueeze(1))

        # Next Q-values from target network
        with torch.no_grad():
            next_q_values, _ = self.target_network(next_states, next_energies)
            target_q_values = rewards.unsqueeze(1) + (
                        self.gamma * next_q_values.max(1)[0].unsqueeze(1) * ~dones.unsqueeze(1))

        # Compute loss and update
        loss = nn.MSELoss()(current_q_values, target_q_values)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        # Update target network periodically
        self.update_counter += 1
        if self.update_counter % 100 == 0:
            self.target_network.load_state_dict(self.q_network.state_dict())

    def compute_reward(self, current_pos, next_pos):
        """Compute reward for Q-learning"""
        # Coverage reward - heavily reward new cell exploration
        if self.map[next_pos] == 'u':
            coverage_reward = 5.0  # Higher reward for visiting unvisited cell
        elif self.map[next_pos] == 'e':
            coverage_reward = -2.0  # Strong penalty for revisiting
        else:
            coverage_reward = 0.0

        # Safety reward (energy constraint)
        if self.check_enough_energy(next_pos):
            safety_reward = 0.1
        else:
            safety_reward = -5.0  # Strong penalty for energy violation

        # Movement reward - encourage actual movement
        if next_pos != current_pos:
            movement_reward = 0.5
        else:
            movement_reward = -1.0  # Penalty for staying in same place

        # Distance from obstacles reward - stay away from new obstacles
        obstacle_penalty = 0.0
        for obs_pos in self.dynamic_obstacles:
            dist_to_obs = math.dist(next_pos, obs_pos)
            if dist_to_obs < 2.0:  # Too close to obstacle
                obstacle_penalty -= 1.0 / (dist_to_obs + 0.1)

        return coverage_reward + safety_reward + movement_reward + obstacle_penalty

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

                # FN (do nothing until close window)
                continue

            # Detect new obstacles
            new_obstacles = self.detect_new_obstacles()
            if new_obstacles:
                print(f"New obstacles detected: {new_obstacles}")
            # Simplified approach: BWave with obstacle avoidance
            wp = self.logic.get_wp(self.current_pos)
            if len(wp) == 0:
                continue

            # BWave selects best waypoint
            selected_cell = self.select_from_wp(wp)

            # Check if selected cell hits new obstacle
            if selected_cell in self.dynamic_obstacles:
                # Use simple avoidance instead of Q-learning
                selected_cell = self.avoid_new_obstacle(selected_cell, wp)
                print(f"Obstacle avoidance: redirected to {selected_cell}")

            if selected_cell == self.current_pos:
                self.task()
            else:
                # CP 0
                if self.logic.state == Q.NORMAL:
                    if self.check_enough_energy(selected_cell) == False:
                        self.charge_planning()
                        continue
                    self.move_to(selected_cell)

                # CP l (l > 0)
                elif self.logic.state == Q.DEADLOCK:
                    # self.bug2(selected_cell)
                    # graph = GridMapGraph(self.map)
                    # path, dist = a_star_search(graph, self.current_pos, selected_cell)
                    path, dist = self.logic.cache_path, self.logic.cache_dist
                    print(f"Deadlock ({round(dist, 2)})")

                    deadlock_count += 1
                    if dist > math.sqrt(ROW_COUNT ** 2 + COL_COUNT ** 2) / 4: extreme_deadlock_count += 1

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

        # Store experience for Q-learning if in unknown region
        if self.new_obstacles_detected and self.last_state is not None:
            reward = self.compute_reward(self.current_pos, pos)
            next_state = self.get_q_learning_state()
            next_energy = torch.FloatTensor([self.energy / ENERGY_CAPACITY]).unsqueeze(0)
            done = (self.logic.state == Q.FINISH)

            experience = (self.last_state, self.last_energy, self.last_action,
                          reward, next_state, next_energy, done)
            self.replay_buffer.append(experience)

            # Learn from experience
            if len(self.replay_buffer) > 100:
                self.learn_from_experience()

        # Store current state for next experience
        if self.new_obstacles_detected:
            self.last_state = self.get_q_learning_state()
            self.last_energy = torch.FloatTensor([self.energy / ENERGY_CAPACITY]).unsqueeze(0)

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

        # cost of travel distance, turning rad
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
        # retreat
        self.retreat()

        # charge
        self.charge()
        time.sleep(0.1)

        # advance
        self.advance()

        # coverage
        self.move_status = 0

    def retreat(self):
        return_path = get_return_path(return_matrix, self.current_pos)
        self.cache_path = return_path  # save for reuse in advance path

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
            # time.sleep(time_delay) # comment to temporary remove delay

            if stop_on_unexpored:
                if self.logic.weight_map[pos] > 0: return

        # self.cache_path.clear()

    # TEST
    def get_better_wp(self, wp):
        if len(wp) == 1: return wp

        new_wp = []
        x_up, y_up = min(wp, key=lambda x: x[0])
        x_down, y_down = max(wp, key=lambda x: x[0])

        if not check_valid_pos((x_up - 1, y_up)) or self.map[(x_up - 1, y_up)] in ('o', 'e'):
            # if abs(self.angle - self.get_angle((x_up, y_up))) <= math.pi / 2:
            new_wp.append((x_up, y_up))
        if not check_valid_pos((x_down + 1, y_down)) or self.map[(x_down + 1, y_down)] in ('o', 'e'):
            # if abs(self.angle - self.get_angle((x_down, y_down))) <= math.pi / 2:
            new_wp.append((x_down, y_down))
        return new_wp

    def set_special_areas(self, special_areas):
        self.logic.set_special_areas(special_areas)

        # TEST
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
    # robot.etm.save_map('potential_field.txt')
    # robot.etm.read_map('potential_field.txt')

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

    # Q-learning statistics
    # Obstacle avoidance statistics
    if len(robot.dynamic_obstacles) > 0:
        print(f'New obstacles detected and avoided: {len(robot.dynamic_obstacles)}')
        print(f'Obstacle positions: {list(robot.dynamic_obstacles)}')

        # Check coverage efficiency
        total_free_cells = np.sum(robot.known_environment == 0) - len(robot.dynamic_obstacles)
        covered_cells = np.sum(robot.map == 'e')
        coverage_percentage = (covered_cells / total_free_cells) * 100
        print(f'Coverage efficiency: {coverage_percentage:.2f}%')

    # with open('result.csv','a',newline='') as fd:
    #     writer = csv.writer(fd)
    #     col_list = [file_name[15:], coverage_length, retreat_length, advance_length, total_travel_length, return_charge_count, execute_time]
    #     writer.writerow(col_list)


if __name__ == "__main__":
    main()