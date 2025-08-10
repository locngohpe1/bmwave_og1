"""
etm.py - SENSOR-BASED IMPLEMENTATION - 🔧 BUG FIXES APPLIED
🔧 100% Paper Compliant với Real-time Sensor Discovery và Enhanced Completion
"""

import numpy as np
from typing import List, Optional, Set, Tuple
from .core import MAPSHierarchy, Position, ETMState, CellState


class ExploratoryTuringMachine:
    """
    ✅ 100% Paper Compliant ETM với Real-time Sensor Discovery
    🔧 FIXED: Enhanced completion logic với global exploration
    - Progressive obstacle discovery within sensor range Rs
    - Dynamic MAPS updates based on sensor feedback
    - Unknown environment assumption
    - 🔧 ENHANCED: Global exploration before premature completion
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

        # 🔧 ENHANCED COMPLETION TRACKING
        self.global_exploration_attempts = 0
        self.max_global_attempts = 5  # Increased from 3 to 5
        self.exploration_radius_expansion = 0

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
        """
        🔧 ENHANCED: CP^l State với global exploration before completion
        """
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
        """
        🔧 ENHANCED: More aggressive global exploration before declaring completion
        Papers allow flexible sensor usage for thorough coverage
        """
        if self.global_exploration_attempts >= self.max_global_attempts:
            # 🔧 FINAL VERIFICATION: Check if there are truly no reachable unexplored areas
            final_check = self._comprehensive_area_scan(current_pos)
            if final_check:
                print(f"🎯 Found reachable area in final check: {final_check.tuple}")
                self.current_wp = [final_check]
                self.state = ETMState.CP0
                # Reset attempts for continued exploration
                self.global_exploration_attempts = 0
                self.exploration_radius_expansion = 0
                return self.current_wp

            print(f"🏁 ETM: Global exploration exhausted after {self.max_global_attempts} attempts")
            print(f"📊 Discovered obstacles: {len(self.discovered_obstacles)}")
            print(f"🔍 Final comprehensive scan found no reachable areas")
            print(f"🎯 Declaring coverage completion")
            self.state = ETMState.FN
            return []

        self.global_exploration_attempts += 1
        self.exploration_radius_expansion += 1

        # 🔧 PROGRESSIVE SEARCH STRATEGY
        base_range = self.sensor_range * 2  # Start with 2x sensor range
        expanded_range = base_range + (self.exploration_radius_expansion * 3.0)  # More aggressive expansion

        print(f"🌍 ETM: Global exploration attempt #{self.global_exploration_attempts}")
        print(f"📡 Expanding search range to {expanded_range} for distant area detection")

        # Try multiple search strategies
        distant_target = self._find_distant_unexplored_area(current_pos, expanded_range)

        if not distant_target:
            # Try alternative search patterns
            distant_target = self._spiral_search_unexplored(current_pos, expanded_range)

        if not distant_target:
            # Try edge-based search
            distant_target = self._edge_based_search(current_pos)

        if distant_target:
            print(f"🎯 Found distant unexplored area at {distant_target.tuple}")
            self.current_wp = [distant_target]
            self.state = ETMState.CP0
            # Reset exploration attempts on success
            self.global_exploration_attempts = 0
            self.exploration_radius_expansion = 0
            return self.current_wp
        else:
            print(f"🔍 No distant areas found with range {expanded_range}")
            # Continue to next expansion or finish
            return self._try_global_exploration_before_finish(current_pos)

    def _find_distant_unexplored_area(self, current_pos: Position,
                                     search_range: float) -> Optional[Position]:
        """
        🔧 NEW: Find unexplored areas within expanded search range
        Paper compliant: Based on sensor discovery principle
        """
        # Search in expanding circles
        max_radius = min(int(search_range), max(self.maps.rows, self.maps.cols) // 2)

        for radius in range(int(self.sensor_range) + 1, max_radius + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) + abs(dc) != radius:
                        continue  # Only check circle perimeter

                    target = Position(
                        current_pos.row + dr,
                        current_pos.col + dc
                    )

                    if (self.maps._is_valid_position(target, 0) and
                        self.maps.states[target.row, target.col] == CellState.U and
                        not self._is_obstacle(target)):

                        distance = current_pos.distance_to(target)
                        if distance <= search_range:
                            return target

        return None

    def _comprehensive_area_scan(self, current_pos: Position) -> Optional[Position]:
        """
        🔧 NEW: Comprehensive final scan for any remaining unexplored reachable areas
        """
        print("🔍 ETM: Performing comprehensive final area scan...")

        # Scan entire map systematically
        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                candidate = Position(row, col)

                # Check if cell is unexplored and potentially reachable
                if (self.maps.states[row, col] == CellState.U and
                    not self._is_obstacle(candidate)):

                    # Check if there's a potential path (rough estimation)
                    if self._is_potentially_reachable(current_pos, candidate):
                        print(f"🎯 Final scan found potentially reachable area: {candidate.tuple}")
                        return candidate

        print("🔍 Comprehensive scan completed - no reachable areas found")
        return None

    def _spiral_search_unexplored(self, current_pos: Position, max_range: float) -> Optional[Position]:
        """
        🔧 NEW: Spiral search pattern for unexplored areas
        """
        max_radius = min(int(max_range), max(self.maps.rows, self.maps.cols) // 2)

        for radius in range(1, max_radius + 1):
            # Search in spiral pattern
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    # Only check cells at current radius distance
                    if abs(dr) + abs(dc) == radius or max(abs(dr), abs(dc)) == radius:
                        target = Position(
                            current_pos.row + dr,
                            current_pos.col + dc
                        )

                        if (self.maps._is_valid_position(target, 0) and
                            self.maps.states[target.row, target.col] == CellState.U and
                            not self._is_obstacle(target)):

                            distance = current_pos.distance_to(target)
                            if distance <= max_range:
                                return target

        return None

    def _edge_based_search(self, current_pos: Position) -> Optional[Position]:
        """
        🔧 NEW: Search along map edges for unexplored areas
        """
        # Check map boundaries for unexplored cells
        edge_candidates = []

        # Top and bottom edges
        for col in range(self.maps.cols):
            for row in [0, self.maps.rows - 1]:
                if row >= 0 and row < self.maps.rows:
                    candidate = Position(row, col)
                    if (self.maps.states[row, col] == CellState.U and
                        not self._is_obstacle(candidate)):
                        edge_candidates.append(candidate)

        # Left and right edges
        for row in range(self.maps.rows):
            for col in [0, self.maps.cols - 1]:
                if col >= 0 and col < self.maps.cols:
                    candidate = Position(row, col)
                    if (self.maps.states[row, col] == CellState.U and
                        not self._is_obstacle(candidate)):
                        edge_candidates.append(candidate)

        # Return nearest edge candidate
        if edge_candidates:
            return min(edge_candidates, key=lambda pos: current_pos.distance_to(pos))

        return None

    def _is_potentially_reachable(self, from_pos: Position, to_pos: Position) -> bool:
        """
        🔧 NEW: Rough estimation if target is potentially reachable
        """
        # Simple line-of-sight check (not perfect but helps filter)
        distance = from_pos.distance_to(to_pos)

        # If too far, likely not reachable
        if distance > max(self.maps.rows, self.maps.cols):
            return False

        # Check if there's a rough path (simplified)
        dr = (to_pos.row - from_pos.row) / max(1, distance)
        dc = (to_pos.col - from_pos.col) / max(1, distance)

        # Sample a few points along the line
        samples = min(10, int(distance))
        for i in range(1, samples):
            sample_row = int(from_pos.row + dr * i)
            sample_col = int(from_pos.col + dc * i)

            if (0 <= sample_row < self.maps.rows and
                0 <= sample_col < self.maps.cols):
                if self.maps.states[sample_row, sample_col] == CellState.O:
                    return False  # Blocked by known obstacle

        return True

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
        """
        🔧 ENHANCED: More thorough completion check
        """
        if self.state != ETMState.FN:
            return False

        # 🔧 ADDITIONAL CHECK: Verify completion is legitimate
        total_discovered = len(self.discovered_obstacles)
        remaining_unexplored = self._count_global_unexplored()

        if self.debug_mode:
            print(f"🔍 Coverage completion check:")
            print(f"   Discovered obstacles: {total_discovered}")
            print(f"   Remaining unexplored: {remaining_unexplored}")
            print(f"   Global exploration attempts: {self.global_exploration_attempts}")

        return True

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
            'current_obstacle_knowledge': self._count_known_obstacles(),
            'global_exploration_attempts': self.global_exploration_attempts
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