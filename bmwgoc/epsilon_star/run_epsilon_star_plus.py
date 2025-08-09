import math
import numpy as np
import pygame as pg
import time
import sys
import os

from grid_map import Grid_Map
from a_star import GridMapGraph, a_star_search
from epsilon_star.epsilon_star_logic import EpsilonStarETM, ETMStates

# Energy capacity
ENERGY_CAPACITY = 1000


class EpsilonStarPlusRobot:
    """
    ε⋆+ Algorithm Robot - Chạy trong môi trường BWave
    """

    def __init__(self, battery_pos, row_count, col_count):
        # Initialize ε⋆ ETM
        self.etm = EpsilonStarETM(row_count, col_count)

        # Robot state
        self.current_pos = battery_pos
        self.battery_pos = battery_pos
        self.energy = ENERGY_CAPACITY

        # Environment for A*
        self.map_for_astar = None

        # Statistics
        self.total_path_length = 0
        self.coverage_length = 0
        self.retreat_length = 0
        self.advance_length = 0
        self.return_count = 0

        # Current segment
        self.segment_type = 'coverage'

    def set_environment(self, environment_array):
        """Set environment"""
        self.etm.set_environment(environment_array)

        # Create map for A*
        self.map_for_astar = {}
        for row in range(len(environment_array)):
            for col in range(len(environment_array[0])):
                if environment_array[row, col] == 0:
                    self.map_for_astar[(row, col)] = 'u'
                else:
                    self.map_for_astar[(row, col)] = 'o'

    def run(self):
        """Main execution loop"""
        clock = pg.time.Clock()
        run = True
        pause = False

        print("Starting ε⋆+ Algorithm...")
        start_time = time.time()

        while run:
            ui.draw()
            clock.tick(60)

            for event in pg.event.get():
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_SPACE:
                        pause = not pause
                        print(f"{'Paused' if pause else 'Resumed'}")
                    elif event.key == pg.K_LEFT:
                        clock.tick(30)  # Slow down
                    elif event.key == pg.K_RIGHT:
                        clock.tick(120)  # Speed up
                elif event.type == pg.QUIT:
                    run = False

            if pause:
                continue

            # Check completion
            if self.etm.is_coverage_complete():
                print("Coverage completed!")
                execution_time = time.time() - start_time
                self._print_statistics(execution_time)
                pause = True
                continue

            # Get waypoint from ε⋆ ETM
            waypoints = self.etm.get_waypoint(self.current_pos)

            if not waypoints:
                continue

            # Select best waypoint
            selected_wp = self._select_waypoint(waypoints)

            if selected_wp == self.current_pos:
                # Task here
                self._perform_task()
            else:
                # Move to waypoint
                if self.segment_type == 'coverage':
                    if not self._check_energy_constraint(selected_wp):
                        self._execute_energy_cycle()
                        continue

                self._move_to(selected_wp)

    def _check_energy_constraint(self, next_pos):
        """Check energy constraint using A⋆"""
        try:
            move_energy = self._calculate_energy(self.current_pos, next_pos, 'coverage')
            retreat_path, retreat_dist = self._calculate_return_path(next_pos)
            retreat_energy = self._calculate_energy_from_distance(retreat_dist, 'retreat')

            return self.energy >= (move_energy + retreat_energy)
        except:
            return False

    def _calculate_return_path(self, from_pos):
        """Calculate return path using A⋆ on visibility graph"""
        graph = GridMapGraph(self.map_for_astar)
        try:
            path, distance = a_star_search(graph, from_pos, self.battery_pos)
            return path, distance
        except:
            # Fallback
            return [from_pos, self.battery_pos], math.dist(from_pos, self.battery_pos)

    def _calculate_energy(self, from_pos, to_pos, segment_type):
        """Calculate energy consumption"""
        distance = math.dist(from_pos, to_pos)
        return self._calculate_energy_from_distance(distance, segment_type)

    def _calculate_energy_from_distance(self, distance, segment_type):
        """Energy model theo paper"""
        if segment_type == 'coverage':
            return 2.0 * distance  # "twice this amount for coverage"
        else:
            return 1.0 * distance  # "proportional for advance/retreat"

    def _execute_energy_cycle(self):
        """Execute retreat-charge-advance cycle"""
        print(f"Energy low ({self.energy:.1f}), returning...")
        self.return_count += 1

        # Retreat
        self._retreat()
        # Charge
        self._charge()
        # Advance
        self._advance()

    def _retreat(self):
        """Retreat to charging station"""
        self.segment_type = 'retreat'
        try:
            retreat_path, _ = self._calculate_return_path(self.current_pos)
            ui.set_charge_path(retreat_path)

            for pos in retreat_path[1:]:
                self._move_to(pos)
        except Exception as e:
            print(f"Retreat error: {e}")

    def _charge(self):
        """Recharge"""
        self.energy = ENERGY_CAPACITY
        ui.set_energy_display(self.energy)

    def _advance(self):
        """Advance to unexplored area"""
        self.segment_type = 'advance'
        target = self._find_advance_target()

        if target:
            try:
                advance_path, _ = self._calculate_return_path(target)
                advance_path = list(reversed(advance_path))
                ui.set_charge_path(advance_path)

                for pos in advance_path[1:]:
                    self._move_to(pos)
            except Exception as e:
                print(f"Advance error: {e}")

        self.segment_type = 'coverage'

    def _find_advance_target(self):
        """Find target for advance"""
        for radius in range(1, 30):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) + abs(dc) != radius:
                        continue

                    target = (self.battery_pos[0] + dr, self.battery_pos[1] + dc)
                    if (target in self.map_for_astar and
                            self.map_for_astar[target] == 'u'):
                        return target
        return None

    def _select_waypoint(self, waypoints):
        """Select best waypoint"""
        if len(waypoints) == 1:
            return waypoints[0]
        return min(waypoints, key=lambda p: math.dist(self.current_pos, p))

    def _move_to(self, new_pos):
        """Move with energy consumption"""
        energy_cost = self._calculate_energy(self.current_pos, new_pos, self.segment_type)

        if self.energy < energy_cost:
            raise Exception("Out of energy!")

        self.energy -= energy_cost
        distance = math.dist(self.current_pos, new_pos)
        self.total_path_length += distance

        if self.segment_type == 'coverage':
            self.coverage_length += distance
            ui.move_to(new_pos)
        elif self.segment_type == 'retreat':
            self.retreat_length += distance
            ui.move_retreat(new_pos)
        elif self.segment_type == 'advance':
            self.advance_length += distance
            ui.move_advance(new_pos)

        self.current_pos = new_pos
        if new_pos in self.map_for_astar:
            self.map_for_astar[new_pos] = 'e'
        ui.set_energy_display(self.energy)

    def _perform_task(self):
        """Perform task"""
        self.etm.task_completed(self.current_pos)
        if self.current_pos in self.map_for_astar:
            self.map_for_astar[self.current_pos] = 'e'
        ui.task(self.current_pos)

    def _print_statistics(self, execution_time):
        """Print final results"""
        print("\n" + "=" * 40)
        print("ε⋆+ ALGORITHM RESULTS")
        print("=" * 40)
        print(f"Coverage:     {self.coverage_length:.2f}")
        print(f"Retreat:      {self.retreat_length:.2f}")
        print(f"Advance:      {self.advance_length:.2f}")
        print(f"Total:        {self.total_path_length:.2f}")
        print(f"Returns:      {self.return_count}")
        print(f"Time:         {execution_time:.3f}s")
        print("=" * 40)


def main():
    """Main function"""
    global ui

    # Initialize UI
    ui = Grid_Map()

    # Read map - you can change this path
    environment, battery_pos = ui.read_map('map/experiment/scenario1/map_1.txt')
    # Or edit map manually:
    # environment, battery_pos = ui.edit_map()

    environment = np.array(environment)
    print(f"Map size: {environment.shape}")
    print(f"Battery at: {battery_pos}")

    # Create and run robot
    robot = EpsilonStarPlusRobot(battery_pos, len(environment), len(environment[0]))
    robot.set_environment(environment)

    print("\nControls:")
    print("SPACE: Pause/Resume")
    print("LEFT:  Slow down")
    print("RIGHT: Speed up")
    print("ESC:   Exit")

    robot.run()
    pg.quit()


if __name__ == "__main__":
    main()