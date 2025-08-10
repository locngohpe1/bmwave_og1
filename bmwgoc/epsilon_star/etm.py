"""
etm.py - SENSOR-BASED IMPLEMENTATION
🔧 100% Paper Compliant với Real-time Sensor Discovery
"""

import numpy as np
from typing import List, Optional, Set, Tuple
from .core import MAPSHierarchy, Position, ETMState, CellState


class ExploratoryTuringMachine:
    """
    ✅ 100% Paper Compliant ETM với Real-time Sensor Discovery
    - Progressive obstacle discovery within sensor range Rs
    - Dynamic MAPS updates based on sensor feedback
    - Unknown environment assumption
    """

    def __init__(self, rows: int, cols: int, sensor_range: float = 2.0):
        self.maps = MAPSHierarchy(rows, cols)
        self.state = ETMState.ST
        self.current_wp = None
        self.prev_wp = None

        # 🔧 SENSOR PARAMETERS per papers
        self.sensor_range = sensor_range  # Rs - detection range
        self.task_range = min(sensor_range, 1.0)  # rt ≤ Rs - task range

        # 🔧 SENSOR-BASED DISCOVERY
        self.discovered_obstacles: Set[Position] = set()
        self.sensor_history: List[Tuple[Position, List[Position]]] = []

        # 🔧 UNKNOWN ENVIRONMENT - Initially all unexplored
        self._initialize_unknown_environment()

        self.debug_mode = True

    def _initialize_unknown_environment(self):
        """
        🌍 Initialize completely unknown environment
        Papers assume robot starts with NO obstacle knowledge
        """
        # All cells start as UNEXPLORED - no obstacle knowledge
        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                self.maps.set_state(Position(row, col), CellState.U)

        self.maps.update_all_levels()
        print(f"🌍 ETM initialized with unknown environment ({self.maps.rows}×{self.maps.cols})")
        print(f"📡 Sensor range Rs = {self.sensor_range}")
        print(f"🛠️  Task range rt = {self.task_range}")

    def get_waypoint(self, current_pos: Position,
                     environment_array: Optional[np.ndarray] = None) -> List[Position]:
        """
        🔧 SENSOR-BASED waypoint computation với progressive discovery

        Args:
            current_pos: Robot's current position
            environment_array: True environment (for sensor simulation)
        """

        # 🔧 SENSOR DISCOVERY PHASE
        if environment_array is not None:
            newly_discovered = self._simulate_sensor_discovery(current_pos, environment_array)
            if newly_discovered:
                print(f"📡 Discovered {len(newly_discovered)} new obstacles at {current_pos.tuple}")

        # ETM State Machine (same as before but with dynamic obstacle knowledge)
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
        """
        📡 Simulate sensor discovery within range Rs

        Papers Quote: "range detectors (e.g. a laser scanner) to detect obstacles
        within a circular region of radius Rs"
        """
        newly_discovered = []

        # Scan circular area within sensor range Rs
        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                sensor_pos = Position(row, col)

                # Check if within sensor range Rs
                distance = current_pos.distance_to(sensor_pos)
                if distance <= self.sensor_range:

                    # Check if this is an obstacle in true environment
                    if (environment_array[row, col] == 1 and
                        sensor_pos not in self.discovered_obstacles):

                        # Newly discovered obstacle!
                        self.discovered_obstacles.add(sensor_pos)
                        newly_discovered.append(sensor_pos)

                        # Update MAPS immediately
                        self.maps.set_state(sensor_pos, CellState.O)

                        if self.debug_mode:
                            print(f"🔍 Sensor discovered obstacle at {sensor_pos.tuple} "
                                  f"(distance: {distance:.1f})")

        # Update MAPS if new obstacles found
        if newly_discovered:
            self.maps.update_all_levels()

            # Record sensor history for ETM input vector ol
            self.sensor_history.append((current_pos, newly_discovered.copy()))

        return newly_discovered

    def _handle_start_state(self) -> List[Position]:
        """
        ST State: Initialize MAPS với unknown environment
        Papers: All cells start as unexplored (U)
        """
        # Environment already initialized as unknown
        self.state = ETMState.CP0

        if self.debug_mode:
            print(f"ETM: ST → CP0 (Unknown environment initialized)")

        return []

    def _handle_cp0_state(self, current_pos: Position) -> List[Position]:
        """
        CP0 State: Compute waypoint với DYNAMIC obstacle knowledge
        """
        self.maps.update_level_0()

        # Get local neighborhood N^0(λ)
        neighborhood = self.maps.get_local_neighborhood(current_pos, 0)

        # Form computing set D^0 với CURRENT obstacle knowledge
        computing_set = self._form_computing_set_level_0(current_pos, neighborhood)

        # Algorithm 1 implementation (same logic, but dynamic obstacles)
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
        """CP^l State: Same logic but with dynamic obstacle knowledge"""
        if level > self.maps.max_level:
            self.state = ETMState.FN
            return []

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
                    self.state = ETMState.FN
                    return []
        else:
            if level < 2:
                self.state = ETMState.CP2 if level == 1 else ETMState.FN
                return self.get_waypoint(current_pos)
            else:
                self.state = ETMState.FN
                return []

    def _handle_wait_state(self, current_pos: Position) -> List[Position]:
        """WT State: Wait for task completion"""
        return [current_pos]

    def _filter_obstacle_waypoints(self, waypoints: List[Position]) -> List[Position]:
        """Filter out known obstacles (only discovered ones)"""
        if not waypoints:
            return []

        valid_waypoints = []
        for wp in waypoints:
            if not self._is_obstacle(wp):
                valid_waypoints.append(wp)
            else:
                if self.debug_mode:
                    print(f"ETM: Filtered discovered obstacle waypoint {wp.tuple}")

        return valid_waypoints

    def _is_obstacle(self, pos: Position) -> bool:
        """
        Check if position is DISCOVERED obstacle
        Only known obstacles (within sensor range) are considered
        """
        if not self.maps._is_valid_position(pos, 0):
            return True

        state = self.maps.states[pos.row, pos.col]
        return state == CellState.O

    def _form_computing_set_level_0(self, current_pos: Position,
                                   neighborhood: List[Position]) -> List[Position]:
        """Form D^0 với current obstacle knowledge"""
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
        """Definition 3.2: Directly reachable if not obstructed by KNOWN obstacles"""
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

        if self.debug_mode:
            remaining = self._count_global_unexplored()
            print(f"ETM: Task completed at {pos.tuple}, {remaining} cells remaining")

    def _count_global_unexplored(self) -> int:
        """Count unexplored cells (only known free cells)"""
        count = 0
        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                if self.maps.states[row, col] == CellState.U:
                    count += 1
        return count

    def set_environment_for_simulation(self, environment_array: np.ndarray):
        """
        🔧 Set true environment for sensor simulation
        Note: ETM doesn't know this - only used for sensor discovery
        """
        self.true_environment = environment_array
        print(f"🌍 True environment set for sensor simulation")
        print(f"📊 Total obstacles in environment: {np.sum(environment_array == 1)}")

    def is_coverage_complete(self) -> bool:
        """Check if coverage complete"""
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
            'discovery_events': len(self.sensor_history),
            'current_obstacle_knowledge': self._count_known_obstacles()
        }

    def _count_known_obstacles(self) -> int:
        """Count currently known obstacles"""
        count = 0
        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                if self.maps.states[row, col] == CellState.O:
                    count += 1
        return count

    def get_coverage_statistics(self) -> dict:
        """Get detailed coverage statistics"""
        total_cells = self.maps.rows * self.maps.cols
        unexplored = self._count_global_unexplored()
        known_obstacles = self._count_known_obstacles()
        explored_count = 0

        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                state = self.maps.states[row, col]
                if state == CellState.E:
                    explored_count += 1

        # Coverage based on known free space only
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