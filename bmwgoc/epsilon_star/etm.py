"""
epsilon_star/etm.py
Exploratory Turing Machine (ETM) implementation
Theo Definition 3.1 và Algorithm 1 từ paper
"""

import numpy as np
from typing import List, Optional
from .core import MAPSHierarchy, Position, ETMState, CellState


class ExploratoryTuringMachine:
    """
    ETM Implementation theo Definition 3.1 và Algorithm 1
    """

    def __init__(self, rows: int, cols: int):
        self.maps = MAPSHierarchy(rows, cols)
        self.state = ETMState.ST
        self.current_wp = None
        self.prev_wp = None

    def get_waypoint(self, current_pos: Position,
                     obstacle_locations: Optional[List[Position]] = None) -> List[Position]:
        """
        Main ETM control function δ
        Implementation của Algorithm 1: Update wp(k)
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
        return []

    def _handle_cp0_state(self, current_pos: Position) -> List[Position]:
        """
        CP0 State: Compute waypoint at Level 0
        Implementation của Algorithm 1 Lines 1-12
        """
        self.maps.update_level_0()

        # Get local neighborhood N^0(λ)
        neighborhood = self.maps.get_local_neighborhood(current_pos, 0)

        # Form computing set D^0 theo Equation (4)
        computing_set = self._form_computing_set_level_0(current_pos, neighborhood)

        # Algorithm 1 implementation
        λ = current_pos

        if λ in computing_set:  # Line 3: current cell is unexplored
            λ_up = Position(λ.row - 1, λ.col)
            λ_down = Position(λ.row + 1, λ.col)

            if λ_up in computing_set and λ_down in computing_set:  # Line 4
                # Line 5: wp(k) = {λ_up, λ_down}
                self.current_wp = [λ_up, λ_down]
                return self.current_wp
            else:
                # Line 6: set λ as waypoint and start tasking
                self.current_wp = [λ]
                self.state = ETMState.WT
                return self.current_wp

        elif computing_set:  # Line 7: other eligible ε-cells exist
            # Line 8: pick ones with max potential
            max_potential_cells = self._get_argmax_potential(computing_set, 0)
            self.current_wp = max_potential_cells
            return self.current_wp

        else:
            # Line 9-11: Check pre-computed wp
            if (self.prev_wp and
                    self.maps.get_potential(self.prev_wp[0], 0) > 0):
                self.current_wp = self.prev_wp
                return self.current_wp
            else:
                # Line 11: local extremum detected
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

            self.current_wp = [target_epsilon_cell]
            self.state = ETMState.CP0
            return self.current_wp
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

    def _form_computing_set_level_0(self, current_pos: Position,
                                    neighborhood: List[Position]) -> List[Position]:
        """
        Form D^0 theo Equation (4):
        D^0 = {α^0 ∈ N^0(λ) : E_α0 > 0, α^0 ∈ DR(λ)}
        """
        computing_set = []

        for pos in neighborhood:
            potential = self.maps.get_potential(pos, 0)
            if potential > 0 and self._is_directly_reachable(current_pos, pos):
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
        Definition 3.2: Directly Reachable Set
        Simplified: check if target is not obstacle
        """
        if not self.maps._is_valid_position(to_pos, 0):
            return False

        state = self.maps.states[to_pos.row, to_pos.col]
        return state != CellState.O

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

    def set_environment(self, environment_array: np.ndarray):
        """Set initial environment (1=obstacle, 0=free)"""
        for row in range(len(environment_array)):
            for col in range(len(environment_array[0])):
                if environment_array[row, col] == 1:
                    self.maps.set_state(Position(row, col), CellState.O)

        self.maps.update_all_levels()

    def is_coverage_complete(self) -> bool:
        """Check if coverage complete (state = FN)"""
        return self.state == ETMState.FN

    def get_state(self) -> ETMState:
        """Get current ETM state"""
        return self.state