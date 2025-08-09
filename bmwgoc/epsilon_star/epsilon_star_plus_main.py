

import sys
import os
import math
import numpy as np
import pygame as pg
import time

from grid_map import Grid_Map
from a_star import GridMapGraph, a_star_search
from .epsilon_star_logic import EpsilonStarETM, ETMStates
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Energy capacity theo paper
ENERGY_CAPACITY = 1000


class EpsilonStarPlusRobot:
    """
    ε⋆+ Algorithm Robot
    "The ε⋆+ algorithm is an extension of the ε⋆ algorithm to consider
    the energy constraints on autonomous vehicles"
    """

    def __init__(self, battery_pos, row_count, col_count):
        # Initialize ε⋆ ETM
        self.etm = EpsilonStarETM(row_count, col_count)

        # Robot state
        self.current_pos = battery_pos
        self.battery_pos = battery_pos
        self.energy = ENERGY_CAPACITY

        # Environment representation for A* pathfinding
        self.map_for_astar = None

        # Trajectory tracking
        self.current_trajectory = None
        self.trajectory_count = 0

        # Statistics
        self.total_path_length = 0
        self.coverage_length = 0
        self.retreat_length = 0
        self.advance_length = 0
        self.return_count = 0

        # Current segment type
        self.segment_type = 'coverage'  # 'coverage', 'retreat', 'advance'

    def set_environment(self, environment_array):
        """
        Set environment from array
        1 = obstacle, 0 = free
        """
        self.etm.set_environment(environment_array)

        # Create map for A* pathfinding
        self.map_for_astar = {}
        for row in range(len(environment_array)):
            for col in range(len(environment_array[0])):
                if environment_array[row, col] == 0:
                    self.map_for_astar[(row, col)] = 'u'  # unvisited
                else:
                    self.map_for_astar[(row, col)] = 'o'  # obstacle

    def run(self):
        """
        Main execution loop cho ε⋆+ algorithm
        """
        clock = pg.time.Clock()
        run = True
        pause = False

        print("Starting ε⋆+ Algorithm...")
        start_time = time.time()

        while run:
            ui.draw()
            clock.tick(60)  # 60 FPS

            # Handle events
            for event in pg.event.get():
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_SPACE:
                        pause = not pause
                        print(f"{'Paused' if pause else 'Resumed'}")
                    elif event.key == pg.K_s:
                        # Save screenshot
                        pg.image.save(ui.WIN, 'epsilon_star_screenshot.png')
                        print("Screenshot saved")
                elif event.type == pg.QUIT:
                    run = False

            if pause:
                continue

            # Check if coverage complete
            if self.etm.is_coverage_complete():
                print("Coverage completed!")
                execution_time = time.time() - start_time
                self._print_final_statistics(execution_time)
                pause = True
                continue

            # Get waypoint from ε⋆ ETM
            waypoint_candidates = self.etm.get_waypoint(self.current_pos)

            if not waypoint_candidates:
                continue

            # Select waypoint using cost function (Equation 5 từ paper)
            selected_waypoint = self._select_waypoint(waypoint_candidates)

            if selected_waypoint == self.current_pos:
                # Task at current position
                self._perform_task()
            else:
                # Move to selected waypoint
                if self.segment_type == 'coverage':
                    # Check energy constraint trước khi move
                    if not self._check_energy_constraint(selected_waypoint):
                        self._execute_energy_constrained_cycle()
                        continue

                self._move_to(selected_waypoint)

    def _check_energy_constraint(self, next_waypoint):
        """
        Energy constraint checking theo ε⋆+ paper:
        "the retreat trajectory is computed from this waypoint to the charging station,
        as the shortest path. Specifically, the visibility graph is extracted from the
        latest symbolically encoded tiling structure"
        """
        try:
            # Calculate energy for next move
            move_energy = self._calculate_move_energy(self.current_pos, next_waypoint)

            # Calculate return path using visibility graph + A⋆
            retreat_path, retreat_distance = self._calculate_retreat_trajectory(next_waypoint)
            retreat_energy = self._calculate_retreat_energy(retreat_distance)

            total_energy_needed = move_energy + retreat_energy

            # Energy constraint check
            return self.energy >= total_energy_needed

        except Exception as e:
            print(f"Energy constraint check failed: {e}")
            return False

    def _calculate_retreat_trajectory(self, from_waypoint):
        """
        Calculate retreat trajectory theo paper:
        "the visibility graph is extracted from the latest symbolically encoded tiling structure.
        In this graph, the node set consists of the charging station, the computed navigation
        waypoint, and the vertices of the obstacles"
        """
        # Create GridMapGraph for A⋆ algorithm
        graph = GridMapGraph(self.map_for_astar)

        try:
            # A⋆ search on visibility graph
            path, distance = a_star_search(graph, from_waypoint, self.battery_pos)
            return path, distance
        except Exception as e:
            # Fallback: direct distance if A⋆ fails
            print(f"A⋆ search failed: {e}, using direct distance")
            direct_distance = math.dist(from_waypoint, self.battery_pos)
            return [from_waypoint, self.battery_pos], direct_distance

    def _calculate_move_energy(self, from_pos, to_pos):
        """
        Calculate energy consumption for move
        Energy model theo paper: "proportional to trajectory length"
        """
        distance = math.dist(from_pos, to_pos)

        if self.segment_type == 'coverage':
            # "twice this amount for the coverage segment"
            return 2.0 * distance
        else:
            # "proportional to trajectory length for advance and retreat segments"
            return 1.0 * distance

    def _calculate_retreat_energy(self, retreat_distance):
        """
        Calculate energy for retreat segment
        """
        return 1.0 * retreat_distance  # Retreat uses base rate

    def _execute_energy_constrained_cycle(self):
        """
        Execute retreat-charge-advance cycle when energy constraint violated
        """
        print(f"Energy low ({self.energy:.1f}), executing retreat-charge-advance cycle...")
        self.return_count += 1

        # 1. Retreat segment
        self._execute_retreat_segment()

        # 2. Charge
        self._execute_charge()

        # 3. Advance segment
        self._execute_advance_segment()

    def _execute_retreat_segment(self):
        """
        Retreat segment: "brings the vehicle back to the charging station along the shortest path"
        """
        print("Executing retreat segment...")
        self.segment_type = 'retreat'

        try:
            retreat_path, _ = self._calculate_retreat_trajectory(self.current_pos)
            ui.set_charge_path(retreat_path)

            # Follow retreat path
            for next_pos in retreat_path[1:]:  # Skip current position
                self._move_to(next_pos)

        except Exception as e:
            print(f"Retreat failed: {e}")

    def _execute_charge(self):
        """
        Recharge at charging station
        """
        print("Recharging...")
        self.energy = ENERGY_CAPACITY
        ui.set_energy_display(self.energy)

    def _execute_advance_segment(self):
        """
        Advance segment: "takes the vehicle from the charging station to an unexplored cell"
        """
        print("Executing advance segment...")
        self.segment_type = 'advance'

        # Find nearby unexplored cell
        target_cell = self._find_advance_target()

        if target_cell:
            try:
                advance_path, _ = self._calculate_retreat_trajectory(target_cell)
                advance_path = list(reversed(advance_path))  # Reverse for advance
                ui.set_charge_path(advance_path)

                # Follow advance path
                for next_pos in advance_path[1:]:  # Skip charging station
                    self._move_to(next_pos)

            except Exception as e:
                print(f"Advance failed: {e}")

        self.segment_type = 'coverage'  # Back to coverage

    def _find_advance_target(self):
        """
        Find target cell for advance segment
        Basic strategy: find nearby unexplored cell
        """
        # Search in expanding radius from charging station
        for radius in range(1, min(50, max(len(self.map_for_astar), 30))):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) + abs(dc) != radius:  # Only check perimeter
                        continue

                    target = (self.battery_pos[0] + dr, self.battery_pos[1] + dc)

                    if (target in self.map_for_astar and
                            self.map_for_astar[target] == 'u'):
                        return target

        return None

    def _select_waypoint(self, waypoint_candidates):
        """
        Select waypoint using cost function
        Equation (5) từ paper: Cμ,λ = dμ,λCTr + θμ,λCTu
        """
        if len(waypoint_candidates) == 1:
            return waypoint_candidates[0]

        # Simple cost function: minimize distance + turning angle
        best_waypoint = waypoint_candidates[0]
        min_cost = float('inf')

        for waypoint in waypoint_candidates:
            # Distance cost
            distance_cost = math.dist(self.current_pos, waypoint)

            # Simplified cost (no turning angle for simplicity)
            total_cost = distance_cost

            if total_cost < min_cost:
                min_cost = total_cost
                best_waypoint = waypoint

        return best_waypoint

    def _move_to(self, new_pos):
        """
        Move robot to new position with energy consumption
        """
        # Calculate energy consumption
        energy_cost = self._calculate_move_energy(self.current_pos, new_pos)

        if self.energy < energy_cost:
            raise Exception("Robot ran out of energy!")

        # Consume energy
        self.energy -= energy_cost

        # Update path length statistics
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

        # Update position
        self.current_pos = new_pos

        # Update map for A*
        if new_pos in self.map_for_astar:
            self.map_for_astar[new_pos] = 'e'  # explored

        # Update UI
        ui.set_energy_display(self.energy)

    def _perform_task(self):
        """
        Perform task at current position
        """
        print(f"Tasking at {self.current_pos}")

        # Update ETM
        self.etm.task_completed(self.current_pos)

        # Update map
        if self.current_pos in self.map_for_astar:
            self.map_for_astar[self.current_pos] = 'e'

        # Update UI
        ui.task(self.current_pos)

    def _print_final_statistics(self, execution_time):
        """
        Print final statistics
        """
        print("\n" + "=" * 50)
        print("ε⋆+ ALGORITHM RESULTS")
        print("=" * 50)
        print(f"Coverage length:    {self.coverage_length:.2f}")
        print(f"Retreat length:     {self.retreat_length:.2f}")
        print(f"Advance length:     {self.advance_length:.2f}")
        print(f"Total path length:  {self.total_path_length:.2f}")
        print(f"Number of returns:  {self.return_count}")
        print(f"Execution time:     {execution_time:.3f}s")

        # Calculate overlap rate
        total_free_cells = sum(1 for cell_state in self.map_for_astar.values()
                               if cell_state in ['e', 'u'])
        explored_cells = sum(1 for cell_state in self.map_for_astar.values()
                             if cell_state == 'e')

        if explored_cells > 0:
            # Simple overlap calculation (approximate)
            estimated_cells_traversed = self.coverage_length
            overlap_rate = max(0, (estimated_cells_traversed - explored_cells) / explored_cells * 100)
            print(f"Overlap rate:       {overlap_rate:.2f}%")

        print("=" * 50)


def main():
    """
    Main function to run ε⋆+ algorithm
    """
    global ui

    # Initialize UI
    ui = Grid_Map()

    # Load map
    map_file = '../map/experiment/scenario1/map_1.txt'
    if not os.path.exists(map_file):
        print(f"Map file not found: {map_file}")
        return

    environment_array, battery_pos = ui.read_map(map_file)
    environment_array = np.array(environment_array)

    print(f"Loaded map: {environment_array.shape}")
    print(f"Battery position: {battery_pos}")

    # Create robot
    robot = EpsilonStarPlusRobot(battery_pos, len(environment_array), len(environment_array[0]))
    robot.set_environment(environment_array)

    # Run algorithm
    robot.run()

    # Keep window open
    pg.quit()


if __name__ == "__main__":
    main()