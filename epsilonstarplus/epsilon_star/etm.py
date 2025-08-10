import numpy as np
from typing import List, Optional, Set, Tuple
from .core import MAPSHierarchy, Position, ETMState, CellState


class ExploratoryTuringMachine:
    """
    Exploratory Turing Machine per Definition 3.1
    Progressive obstacle discovery with sensor range Rs
    """

    def __init__(self, rows: int, cols: int, sensor_range: float = 2.0):
        self.maps = MAPSHierarchy(rows, cols)
        self.state = ETMState.ST
        self.current_wp = None
        self.prev_wp = None

        self.sensor_range = sensor_range
        self.task_range = min(sensor_range, 1.0)

        self.discovered_obstacles: Set[Position] = set()
        self.global_exploration_attempts = 0
        self.max_global_attempts = 5

        self._initialize_unknown_environment()

    def _initialize_unknown_environment(self):
        """Initialize completely unknown environment"""
        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                self.maps.set_state(Position(row, col), CellState.U)
        self.maps.update_all_levels()

    def get_waypoint(self, current_pos: Position,
                     environment_array: Optional[np.ndarray] = None) -> List[Position]:
        """Get waypoint with sensor discovery"""
        if environment_array is not None:
            self._simulate_sensor_discovery(current_pos, environment_array)

        if self.state == ETMState.ST:
            return self._handle_start_state()
        elif self.state == ETMState.CP0:
            return self._handle_cp0_state(current_pos)
        elif self.state in [ETMState.CP1, ETMState.CP2]:
            level = 1 if self.state == ETMState.CP1 else 2
            return self._handle_cpl_state(current_pos, level)
        elif self.state == ETMState.WT:
            return self._handle_wait_state(current_pos)
        elif self.state == ETMState.FN:
            return []

        return []

    def _simulate_sensor_discovery(self, current_pos: Position,
                                 environment_array: np.ndarray) -> List[Position]:
        """Simulate sensor discovery within range Rs"""
        newly_discovered = []

        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                sensor_pos = Position(row, col)
                distance = current_pos.distance_to(sensor_pos)

                if (distance <= self.sensor_range and
                    environment_array[row, col] == 1 and
                    sensor_pos not in self.discovered_obstacles):

                    self.discovered_obstacles.add(sensor_pos)
                    newly_discovered.append(sensor_pos)
                    self.maps.set_state(sensor_pos, CellState.O)

        if newly_discovered:
            self.maps.update_all_levels()

        return newly_discovered

    def _handle_start_state(self) -> List[Position]:
        """ST State: Initialize MAPS"""
        self.state = ETMState.CP0
        return []

    def _handle_cp0_state(self, current_pos: Position) -> List[Position]:
        """CP0 State: Algorithm 1 implementation"""
        self.maps.update_level_0()
        neighborhood = self.maps.get_local_neighborhood(current_pos, 0)
        computing_set = self._form_computing_set_level_0(current_pos, neighborhood)

        λ = current_pos

        if λ in computing_set:
            λ_up = Position(λ.row - 1, λ.col)
            λ_down = Position(λ.row + 1, λ.col)

            if λ_up in computing_set and λ_down in computing_set:
                valid_waypoints = self._filter_obstacle_waypoints([λ_up, λ_down])
                if valid_waypoints:
                    self.current_wp = valid_waypoints
                    return self.current_wp
                else:
                    self.current_wp = [λ]
                    self.state = ETMState.WT
                    return self.current_wp
            else:
                self.current_wp = [λ]
                self.state = ETMState.WT
                return self.current_wp

        elif computing_set:
            max_potential_cells = self._get_argmax_potential(computing_set, 0)
            valid_waypoints = self._filter_obstacle_waypoints(max_potential_cells)
            if valid_waypoints:
                self.current_wp = valid_waypoints
                return self.current_wp
            else:
                self.current_wp = None
                self.state = ETMState.CP1
                return self.get_waypoint(current_pos)

        else:
            if (self.prev_wp and len(self.prev_wp) > 0):
                valid_prev_wp = self._filter_obstacle_waypoints(self.prev_wp)
                if valid_prev_wp and self.maps.get_potential(valid_prev_wp[0], 0) > 0:
                    self.current_wp = valid_prev_wp
                    return self.current_wp

            self.current_wp = None
            self.state = ETMState.CP1
            return self.get_waypoint(current_pos)

    def _handle_cpl_state(self, current_pos: Position, level: int) -> List[Position]:
        """CP^l State with global exploration"""
        if level > self.maps.max_level:
            return self._try_global_exploration_before_finish(current_pos)

        self.maps.update_level_l(level)
        coarse_pos = self.maps.map_to_coarse_level(current_pos, level)
        neighborhood = self.maps.get_local_neighborhood(coarse_pos, level)
        computing_set = self._form_computing_set_level_l(neighborhood, level)

        if computing_set:
            best_coarse_cell = self._get_argmax_potential(computing_set, level)[0]
            target_epsilon_cell = self.maps.select_epsilon_cell_in_coarse_cell(
                best_coarse_cell, level
            )

            if not self._is_obstacle(target_epsilon_cell):
                self.current_wp = [target_epsilon_cell]
                self.state = ETMState.CP0
                return self.current_wp
            else:
                if level < 2:
                    self.state = ETMState.CP2 if level == 1 else ETMState.FN
                    return self.get_waypoint(current_pos)
                else:
                    return self._try_global_exploration_before_finish(current_pos)
        else:
            if level < 2:
                self.state = ETMState.CP2 if level == 1 else ETMState.FN
                return self.get_waypoint(current_pos)
            else:
                return self._try_global_exploration_before_finish(current_pos)

    def _try_global_exploration_before_finish(self, current_pos: Position) -> List[Position]:
        """Global exploration before declaring completion"""
        if self.global_exploration_attempts >= self.max_global_attempts:
            final_check = self._comprehensive_area_scan(current_pos)
            if final_check:
                self.current_wp = [final_check]
                self.state = ETMState.CP0
                self.global_exploration_attempts = 0
                return self.current_wp

            self.state = ETMState.FN
            return []

        self.global_exploration_attempts += 1
        base_range = self.sensor_range * 2
        expanded_range = base_range + (self.global_exploration_attempts * 3.0)

        distant_target = self._find_distant_unexplored_area(current_pos, expanded_range)

        if not distant_target:
            distant_target = self._spiral_search_unexplored(current_pos, expanded_range)

        if not distant_target:
            distant_target = self._edge_based_search(current_pos)

        if distant_target:
            self.current_wp = [distant_target]
            self.state = ETMState.CP0
            self.global_exploration_attempts = 0
            return self.current_wp
        else:
            return self._try_global_exploration_before_finish(current_pos)

    def _find_distant_unexplored_area(self, current_pos: Position,
                                     search_range: float) -> Optional[Position]:
        """Find unexplored areas within expanded search range"""
        max_radius = min(int(search_range), max(self.maps.rows, self.maps.cols) // 2)

        for radius in range(int(self.sensor_range) + 1, max_radius + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) + abs(dc) != radius:
                        continue

                    target = Position(current_pos.row + dr, current_pos.col + dc)

                    if (self.maps._is_valid_position(target, 0) and
                        self.maps.states[target.row, target.col] == CellState.U and
                        not self._is_obstacle(target)):

                        distance = current_pos.distance_to(target)
                        if distance <= search_range:
                            return target

        return None

    def _comprehensive_area_scan(self, current_pos: Position) -> Optional[Position]:
        """Comprehensive final scan for remaining unexplored areas"""
        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                candidate = Position(row, col)

                if (self.maps.states[row, col] == CellState.U and
                    not self._is_obstacle(candidate) and
                    self._is_potentially_reachable(current_pos, candidate)):
                    return candidate

        return None

    def _spiral_search_unexplored(self, current_pos: Position, max_range: float) -> Optional[Position]:
        """Spiral search pattern for unexplored areas"""
        max_radius = min(int(max_range), max(self.maps.rows, self.maps.cols) // 2)

        for radius in range(1, max_radius + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) + abs(dc) == radius or max(abs(dr), abs(dc)) == radius:
                        target = Position(current_pos.row + dr, current_pos.col + dc)

                        if (self.maps._is_valid_position(target, 0) and
                            self.maps.states[target.row, target.col] == CellState.U and
                            not self._is_obstacle(target)):

                            distance = current_pos.distance_to(target)
                            if distance <= max_range:
                                return target

        return None

    def _edge_based_search(self, current_pos: Position) -> Optional[Position]:
        """Search along map edges for unexplored areas"""
        edge_candidates = []

        for col in range(self.maps.cols):
            for row in [0, self.maps.rows - 1]:
                if row >= 0 and row < self.maps.rows:
                    candidate = Position(row, col)
                    if (self.maps.states[row, col] == CellState.U and
                        not self._is_obstacle(candidate)):
                        edge_candidates.append(candidate)

        for row in range(self.maps.rows):
            for col in [0, self.maps.cols - 1]:
                if col >= 0 and col < self.maps.cols:
                    candidate = Position(row, col)
                    if (self.maps.states[row, col] == CellState.U and
                        not self._is_obstacle(candidate)):
                        edge_candidates.append(candidate)

        if edge_candidates:
            return min(edge_candidates, key=lambda pos: current_pos.distance_to(pos))

        return None

    def _is_potentially_reachable(self, from_pos: Position, to_pos: Position) -> bool:
        """Rough estimation if target is potentially reachable"""
        distance = from_pos.distance_to(to_pos)

        if distance > max(self.maps.rows, self.maps.cols):
            return False

        dr = (to_pos.row - from_pos.row) / max(1, distance)
        dc = (to_pos.col - from_pos.col) / max(1, distance)

        samples = min(10, int(distance))
        for i in range(1, samples):
            sample_row = int(from_pos.row + dr * i)
            sample_col = int(from_pos.col + dc * i)

            if (0 <= sample_row < self.maps.rows and
                0 <= sample_col < self.maps.cols):
                if self.maps.states[sample_row, sample_col] == CellState.O:
                    return False

        return True

    def _handle_wait_state(self, current_pos: Position) -> List[Position]:
        """WT State: Wait for task completion"""
        return [current_pos]

    def _filter_obstacle_waypoints(self, waypoints: List[Position]) -> List[Position]:
        """Filter out discovered obstacles"""
        if not waypoints:
            return []

        valid_waypoints = []
        for wp in waypoints:
            if not self._is_obstacle(wp):
                valid_waypoints.append(wp)

        return valid_waypoints

    def _is_obstacle(self, pos: Position) -> bool:
        """Check if position is discovered obstacle"""
        if not self.maps._is_valid_position(pos, 0):
            return True

        state = self.maps.states[pos.row, pos.col]
        return state == CellState.O

    def _form_computing_set_level_0(self, current_pos: Position,
                                   neighborhood: List[Position]) -> List[Position]:
        """Form D^0 with current obstacle knowledge"""
        computing_set = []

        for pos in neighborhood:
            potential = self.maps.get_potential(pos, 0)
            if (potential > 0 and
                not self._is_obstacle(pos) and
                self._is_directly_reachable(current_pos, pos)):
                computing_set.append(pos)

        return computing_set

    def _form_computing_set_level_l(self, neighborhood: List[Position],
                                   level: int) -> List[Position]:
        """Form D^l for higher levels"""
        computing_set = []

        for pos in neighborhood:
            potential = self.maps.get_potential(pos, level)
            if potential > 0:
                computing_set.append(pos)

        return computing_set

    def _is_directly_reachable(self, from_pos: Position, to_pos: Position) -> bool:
        """Definition 3.2: Directly reachable if not obstructed"""
        if not self.maps._is_valid_position(to_pos, 0):
            return False

        if self._is_obstacle(to_pos):
            return False

        return True

    def _get_argmax_potential(self, cell_list: List[Position],
                             level: int) -> List[Position]:
        """Get cells with maximum potential at given level"""
        if not cell_list:
            return []

        max_potential = float('-inf')
        max_cells = []

        for pos in cell_list:
            potential = self.maps.get_potential(pos, level)
            if potential > max_potential:
                max_potential = potential
                max_cells = [pos]
            elif potential == max_potential:
                max_cells.append(pos)

        return max_cells

    def task_completed(self, pos: Position):
        """Called when task completed at position"""
        self.maps.set_state(pos, CellState.E)
        self.maps.update_level_0()

        if self.state == ETMState.WT:
            self.state = ETMState.CP0

    def set_environment_for_simulation(self, environment_array: np.ndarray):
        """Set true environment for sensor simulation"""
        self.true_environment = environment_array

    def is_coverage_complete(self) -> bool:
        """Check if coverage is complete"""
        return self.state == ETMState.FN

    def get_state(self) -> ETMState:
        """Get current ETM state"""
        return self.state

    def get_sensor_statistics(self) -> dict:
        """Get sensor discovery statistics"""
        return {
            'total_discovered_obstacles': len(self.discovered_obstacles),
            'sensor_range': self.sensor_range,
            'task_range': self.task_range,
            'global_exploration_attempts': self.global_exploration_attempts
        }

    def get_coverage_statistics(self) -> dict:
        """Get detailed coverage statistics"""
        total_cells = self.maps.rows * self.maps.cols
        unexplored = sum(1 for row in range(self.maps.rows)
                        for col in range(self.maps.cols)
                        if self.maps.states[row, col] == CellState.U)
        known_obstacles = len(self.discovered_obstacles)
        explored_count = sum(1 for row in range(self.maps.rows)
                           for col in range(self.maps.cols)
                           if self.maps.states[row, col] == CellState.E)

        known_free_cells = total_cells - known_obstacles
        coverage_percentage = (explored_count / known_free_cells * 100) if known_free_cells > 0 else 0

        return {
            'total_cells': total_cells,
            'known_free_cells': known_free_cells,
            'known_obstacle_cells': known_obstacles,
            'explored_cells': explored_count,
            'unexplored_cells': unexplored,
            'coverage_percentage': coverage_percentage,
            'sensor_statistics': self.get_sensor_statistics()
        }