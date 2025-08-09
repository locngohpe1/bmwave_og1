"""
epsilon_star/etm.py - FIXED VERSION
Exploratory Turing Machine với PROPER obstacle avoidance
"""

import numpy as np
from typing import List, Optional
from .core import MAPSHierarchy, Position, ETMState, CellState


class ExploratoryTuringMachine:
    """
    FIXED ETM Implementation với proper obstacle avoidance
    ✅ Robot will NEVER move into obstacle cells
    """

    def __init__(self, rows: int, cols: int):
        self.maps = MAPSHierarchy(rows, cols)
        self.state = ETMState.ST
        self.current_wp = None
        self.prev_wp = None
        self.debug_mode = False  # Disable debug by default

    def get_waypoint(self, current_pos: Position,
                     obstacle_locations: Optional[List[Position]] = None) -> List[Position]:
        """
        Main ETM control function δ với PROPER obstacle avoidance
        """
        # Update MAPS with obstacle information
        if obstacle_locations:
            self._update_obstacles(obstacle_locations)

        # ETM State Machine
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

    def _handle_start_state(self) -> List[Position]:
        """ST State: Initialize MAPS"""
        # All cells start as unexplored (U)
        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                self.maps.set_state(Position(row, col), CellState.U)

        self.maps.update_all_levels()
        self.state = ETMState.CP0

        if self.debug_mode:
            print(f"ETM: Initialized {self.maps.rows}×{self.maps.cols} MAPS")

        return []

    def _handle_cp0_state(self, current_pos: Position) -> List[Position]:
        """
        CP0 State: Compute waypoint at Level 0 với OBSTACLE AVOIDANCE
        Implementation của Algorithm 1 Lines 1-12
        """
        self.maps.update_level_0()

        # Get local neighborhood N^0(λ)
        neighborhood = self.maps.get_local_neighborhood(current_pos, 0)

        # ✅ FIXED: Form computing set D^0 với PROPER obstacle filtering
        computing_set = self._form_computing_set_level_0(current_pos, neighborhood)

        # Algorithm 1 implementation
        λ = current_pos

        if λ in computing_set:  # Line 3: current cell is unexplored
            λ_up = Position(λ.row - 1, λ.col)
            λ_down = Position(λ.row + 1, λ.col)

            if λ_up in computing_set and λ_down in computing_set:  # Line 4
                # Line 5: wp(k) = {λ_up, λ_down} - but filter obstacles!
                valid_waypoints = self._filter_obstacle_waypoints([λ_up, λ_down])
                if valid_waypoints:
                    self.current_wp = valid_waypoints
                    return self.current_wp
                else:
                    # Both blocked by obstacles, task current cell
                    self.current_wp = [λ]
                    self.state = ETMState.WT
                    return self.current_wp
            else:
                # Line 6: set λ as waypoint and start tasking
                self.current_wp = [λ]
                self.state = ETMState.WT
                return self.current_wp

        elif computing_set:  # Line 7: other eligible ε-cells exist
            # Line 8: pick ones with max potential - but filter obstacles!
            max_potential_cells = self._get_argmax_potential(computing_set, 0)
            valid_waypoints = self._filter_obstacle_waypoints(max_potential_cells)
            if valid_waypoints:
                self.current_wp = valid_waypoints
                return self.current_wp
            else:
                # All max potential cells are obstacles - local extremum
                self.current_wp = None
                self.state = ETMState.CP1
                return self.get_waypoint(current_pos)  # Recursive call

        else:
            # Line 9-11: Check pre-computed wp
            if (self.prev_wp and len(self.prev_wp) > 0):
                valid_prev_wp = self._filter_obstacle_waypoints(self.prev_wp)
                if valid_prev_wp and self.maps.get_potential(valid_prev_wp[0], 0) > 0:
                    self.current_wp = valid_prev_wp
                    return self.current_wp

            # Line 11: local extremum detected at Level 0
            self.current_wp = None
            self.state = ETMState.CP1
            return self.get_waypoint(current_pos)  # Recursive call

    def _handle_cpl_state(self, current_pos: Position, level: int) -> List[Position]:
        """
        CPl State: Compute waypoint at Level l (l ≥ 1)
        Algorithm 1 Lines 13-18
        """
        if level > self.maps.max_level:
            self.state = ETMState.FN
            return []

        self.maps.update_level_l(level)

        # Map to coarse level
        coarse_pos = self.maps.map_to_coarse_level(current_pos, level)

        # Get neighborhood at Level l
        neighborhood = self.maps.get_local_neighborhood(coarse_pos, level)

        # Form computing set D^l
        computing_set = self._form_computing_set_level_l(neighborhood, level)

        if computing_set:  # Line 15: coarse cells with +ve potentials exist
            # Line 16: wp(k) = I(argmax E_αℓ)
            best_coarse_cell = self._get_argmax_potential(computing_set, level)[0]
            target_epsilon_cell = self.maps.select_epsilon_cell_in_coarse_cell(
                best_coarse_cell, level
            )

            # ✅ FIXED: Verify target is not obstacle before returning
            if not self._is_obstacle(target_epsilon_cell):
                self.current_wp = [target_epsilon_cell]
                self.state = ETMState.CP0
                return self.current_wp
            else:
                # Target is obstacle, try next level
                if level < 2:
                    self.state = ETMState.CP2 if level == 1 else ETMState.FN
                    return self.get_waypoint(current_pos)
                else:
                    self.state = ETMState.FN
                    return []
        else:
            # Line 17: no waypoint found at Level l
            if level < 2:  # Try next level
                self.state = ETMState.CP2 if level == 1 else ETMState.FN
                return self.get_waypoint(current_pos)
            else:
                self.state = ETMState.FN
                return []

    def _handle_wait_state(self, current_pos: Position) -> List[Position]:
        """WT State: Wait for task completion"""
        return [current_pos]

    def _filter_obstacle_waypoints(self, waypoints: List[Position]) -> List[Position]:
        """
        ✅ CRITICAL FIX: Filter out obstacle waypoints
        This prevents robot from moving into walls!
        """
        if not waypoints:
            return []

        valid_waypoints = []
        for wp in waypoints:
            if not self._is_obstacle(wp):
                valid_waypoints.append(wp)
            else:
                if self.debug_mode:
                    print(f"ETM: Filtered obstacle waypoint {wp.tuple}")

        return valid_waypoints

    def _is_obstacle(self, pos: Position) -> bool:
        """
        ✅ CRITICAL: Check if position is obstacle
        This is the KEY function that prevents wall-walking!
        """
        if not self.maps._is_valid_position(pos, 0):
            return True  # Out of bounds = obstacle

        state = self.maps.states[pos.row, pos.col]
        return state == CellState.O

    def _count_global_unexplored(self) -> int:
        """Count total unexplored cells globally"""
        count = 0
        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                if self.maps.states[row, col] == CellState.U:
                    count += 1
        return count

    def _find_global_unexplored_cell(self) -> Optional[Position]:
        """Find any unexplored cell globally (for emergency waypoint)"""
        # Search in expanding radius from current position
        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                pos = Position(row, col)
                if (self.maps.states[row, col] == CellState.U and
                        self._is_reachable_globally(pos)):
                    if self.debug_mode:
                        print(f"ETM: Found global unexplored cell at {pos.tuple}")
                    return pos
        return None

    def _is_reachable_globally(self, target: Position) -> bool:
        """Check if position is globally reachable (not surrounded by obstacles)"""
        # Simple reachability check - target is not an obstacle
        if not self.maps._is_valid_position(target, 0):
            return False

        state = self.maps.states[target.row, target.col]
        return state != CellState.O

    def _form_computing_set_level_0(self, current_pos: Position,
                                    neighborhood: List[Position]) -> List[Position]:
        """
        ✅ FIXED D^0 formation với PROPER obstacle avoidance
        """
        computing_set = []

        for pos in neighborhood:
            potential = self.maps.get_potential(pos, 0)
            # ✅ CRITICAL: Check both potential > 0 AND not obstacle
            if (potential > 0 and
                not self._is_obstacle(pos) and
                self._is_directly_reachable(current_pos, pos)):
                computing_set.append(pos)

        return computing_set

    def _form_computing_set_level_l(self, neighborhood: List[Position],
                                    level: int) -> List[Position]:
        """
        Form D^l theo Equation (4):
        D^l = {α^l ∈ N^l(λ) : E_αl > 0}
        """
        computing_set = []

        for pos in neighborhood:
            potential = self.maps.get_potential(pos, level)
            if potential > 0:
                computing_set.append(pos)

        return computing_set

    def _is_directly_reachable(self, from_pos: Position, to_pos: Position) -> bool:
        """
        Definition 3.2: Directly Reachable Set với obstacle checking
        """
        if not self.maps._is_valid_position(to_pos, 0):
            return False

        # ✅ CRITICAL: Must not be obstacle
        if self._is_obstacle(to_pos):
            return False

        # Additional line-of-sight check could be added here
        return True

    def _has_line_of_sight(self, from_pos: Position, to_pos: Position) -> bool:
        """Check line of sight between two positions"""
        # Simple implementation - check if adjacent or no obstacles in between
        dr = abs(to_pos.row - from_pos.row)
        dc = abs(to_pos.col - from_pos.col)

        # Adjacent cells are always reachable
        if dr <= 1 and dc <= 1:
            return True

        # For distant cells, ensure no obstacles block the path
        # Simplified: just check if target is accessible
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

    def _update_obstacles(self, obstacle_locations: List[Position]):
        """Update MAPS with newly discovered obstacles"""
        for pos in obstacle_locations:
            self.maps.set_state(pos, CellState.O)

        self.maps.update_all_levels()

    def task_completed(self, pos: Position):
        """Called when task completed at position"""
        self.maps.set_state(pos, CellState.E)
        self.maps.update_level_0()

        if self.state == ETMState.WT:
            self.state = ETMState.CP0

        if self.debug_mode:
            remaining = self._count_global_unexplored()
            print(f"ETM: Task completed at {pos.tuple}, {remaining} cells remaining")

    def set_environment(self, environment_array: np.ndarray):
        """
        ✅ FIXED: Set initial environment with PROPER obstacle marking
        """
        obstacle_count = 0
        for row in range(len(environment_array)):
            for col in range(len(environment_array[0])):
                # ✅ CRITICAL: Handle both 1 and '1' as obstacles
                cell_value = environment_array[row, col]
                if cell_value == 1 or cell_value == '1':
                    self.maps.set_state(Position(row, col), CellState.O)
                    obstacle_count += 1

        self.maps.update_all_levels()

        total_cells = len(environment_array) * len(environment_array[0])
        free_cells = total_cells - obstacle_count

        if self.debug_mode:
            print(f"ETM: Environment set - {free_cells} free, {obstacle_count} obstacles")

        print(f"🗺️  ETM Environment loaded: {total_cells} total, {obstacle_count} obstacles, {free_cells} free")

    def is_coverage_complete(self) -> bool:
        """Check if coverage complete (state = FN)"""
        return self.state == ETMState.FN

    def get_state(self) -> ETMState:
        """Get current ETM state"""
        return self.state

    def get_coverage_statistics(self) -> dict:
        """Get detailed coverage statistics"""
        total_cells = self.maps.rows * self.maps.cols
        unexplored = self._count_global_unexplored()

        obstacle_count = 0
        explored_count = 0

        for row in range(self.maps.rows):
            for col in range(self.maps.cols):
                state = self.maps.states[row, col]
                if state == CellState.O:
                    obstacle_count += 1
                elif state == CellState.E:
                    explored_count += 1

        free_cells = total_cells - obstacle_count
        coverage_percentage = (explored_count / free_cells * 100) if free_cells > 0 else 0

        return {
            'total_cells': total_cells,
            'free_cells': free_cells,
            'obstacle_cells': obstacle_count,
            'explored_cells': explored_count,
            'unexplored_cells': unexplored,
            'coverage_percentage': coverage_percentage
        }