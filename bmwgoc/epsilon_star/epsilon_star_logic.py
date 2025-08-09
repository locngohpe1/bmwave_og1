import numpy as np
from .maps_hierarchy import MAPSHierarchy


class ETMStates:
    """
    ETM States theo Definition 3.1 từ paper:
    Q = {ST,CP0,...,CPL,WT,FN}
    """
    ST = 0  # Start: Initialize MAPS
    CP0 = 1  # Compute at Level 0
    CP1 = 2  # Compute at Level 1
    CP2 = 3  # Compute at Level 2
    CP3 = 4  # Compute at Level 3
    WT = 10  # Wait: Wait for AV to finish task
    FN = 11  # Finish: Complete coverage


class EpsilonStarETM:
    """
    Exploratory Turing Machine (ETM) implementation
    Theo Definition 3.1 và Algorithm 1 từ paper
    """

    def __init__(self, row_count, col_count):
        # Initialize MAPS
        self.maps = MAPSHierarchy(row_count, col_count)

        # ETM state
        self.state = ETMStates.ST

        # Current waypoint cache
        self._current_wp = None
        self._prev_wp = None

    def get_waypoint(self, current_pos, obstacle_locations=None):
        """
        Main ETM control function δ
        Implementation của Algorithm 1: Update wp(k) từ paper

        Args:
            current_pos: λ - Index of ε-cell where AV is currently located
            obstacle_locations: ol - Vector of obstacle locations

        Returns:
            wp(k): Candidate set of navigation waypoints
        """

        # Update MAPS với obstacle information
        if obstacle_locations:
            self._update_obstacle_information(obstacle_locations)

        # ETM State Machine theo Figure 4
        if self.state == ETMStates.ST:
            return self._handle_start_state()

        elif self.state == ETMStates.CP0:
            return self._handle_cp0_state(current_pos)

        elif self.state in [ETMStates.CP1, ETMStates.CP2, ETMStates.CP3]:
            level = self.state - ETMStates.CP0
            return self._handle_cpl_state(current_pos, level)

        elif self.state == ETMStates.WT:
            return self._handle_wait_state(current_pos)

        elif self.state == ETMStates.FN:
            return []  # Coverage complete

    def _handle_start_state(self):
        """
        ST State: Initialize MAPS
        "Since the whole area is initially unexplored, all ε-cells are assigned the state U"
        """
        # Initialize all cells as unexplored (U)
        for row in range(self.maps.row_count):
            for col in range(self.maps.col_count):
                self.maps.symbolic_states[row, col] = 'U'

        # Update MAPS using field B
        self.maps.update_all_levels()

        # Transition to CP0
        self.state = ETMStates.CP0

        return None  # No waypoint yet

    def _handle_cp0_state(self, current_pos):
        """
        CP0 State: Compute waypoint at Level 0
        Implementation của Algorithm 1 Lines 1-12
        """
        # Update MAPS at Level 0
        self.maps.update_potential_surface_level_0()

        # Get local neighborhood N0(λ)
        local_neighborhood = self.maps.get_local_neighborhood_level_0(current_pos)

        # Form computing set D0 theo equation (4)
        computing_set_D0 = self._form_computing_set_D0(current_pos, local_neighborhood)

        # Algorithm 1 Lines 3-12: Waypoint computation logic
        λ = current_pos

        if λ in computing_set_D0:  # Line 3: current cell λ is unexplored
            λ_up = (λ[0] - 1, λ[1])
            λ_down = (λ[0] + 1, λ[1])

            if λ_up in computing_set_D0 and λ_down in computing_set_D0:  # Line 4
                # Line 5: wp(k) = {λ_up, λ_down} - AV will pick one per Eq. (5)
                self._current_wp = [λ_up, λ_down]
                return self._current_wp
            else:
                # Line 6: set λ as waypoint and start tasking
                self._current_wp = [λ]
                self.state = ETMStates.WT  # Transition to wait state
                return self._current_wp

        elif len(computing_set_D0) > 0:  # Line 7: other eligible ε-cells exist
            # Line 8: pick the ones with max potential
            max_potential_cells = self._get_argmax_potential(computing_set_D0, level=0)
            self._current_wp = max_potential_cells
            return self._current_wp

        else:
            # Line 9-11: Check if pre-computed wp still available
            if (self._prev_wp and
                    self.maps.get_potential_at_level(self._prev_wp[0], 0) > 0):
                self._current_wp = self._prev_wp
                return self._current_wp
            else:
                # Line 11: local extremum detected at Level 0
                self._current_wp = None
                self.state = ETMStates.CP1  # Switch to higher level
                return self.get_waypoint(current_pos)  # Recursive call

    def _handle_cpl_state(self, current_pos, level):
        """
        CPl State: Compute waypoint at Level l (l ≥ 1)
        Implementation của Algorithm 1 Lines 13-18
        """
        if level > self.maps.L:
            # No more levels available
            self.state = ETMStates.FN
            return []

        # Update MAPS at current level
        self.maps.update_potential_surface_level_l(level)

        # Map current position to coarse level
        coarse_pos = self.maps.map_epsilon_cell_to_coarse_level(current_pos, level)

        # Get local neighborhood at Level l
        local_neighborhood = self.maps.get_local_neighborhood_level_l(coarse_pos, level)

        # Form computing set Dl theo equation (4)
        computing_set_Dl = self._form_computing_set_Dl(local_neighborhood, level)

        if len(computing_set_Dl) > 0:  # Line 15: coarse cells with +ve potentials exist
            # Line 16: wp(k) = I(argmax Eαl)
            best_coarse_cell = self._get_argmax_potential(computing_set_Dl, level)[0]
            target_epsilon_cell = self.maps.select_random_epsilon_cell_in_coarse_cell(
                best_coarse_cell, level)

            self._current_wp = [target_epsilon_cell]
            self.state = ETMStates.CP0  # Return to Level 0
            return self._current_wp
        else:
            # Line 17: no waypoint found at Level l
            self._current_wp = None
            next_level_state = getattr(ETMStates, f'CP{level + 1}', None)
            if next_level_state and level + 1 <= 3:  # Max 3 levels for simplicity
                self.state = next_level_state
                return self.get_waypoint(current_pos)  # Try next level
            else:
                self.state = ETMStates.FN  # No more levels
                return []

    def _handle_wait_state(self, current_pos):
        """
        WT State: Wait for task completion
        """
        # Return current position for tasking
        return [current_pos]

    def task_completed(self, pos):
        """
        Called when task is completed at position
        Update state from E to explored and transition back to CP0
        """
        self.maps.set_explored(pos)
        self.maps.update_potential_surface_level_0()

        # Transition back to CP0
        if self.state == ETMStates.WT:
            self.state = ETMStates.CP0

    def _form_computing_set_D0(self, current_pos, neighborhood):
        """
        Form computing set D0 theo equation (4) từ paper:
        D0 = {α0 ∈ N0(λ) : Eα0 > 0, α0 ∈ DR(λ)}

        DR(λ) = directly reachable set (Definition 3.2)
        """
        computing_set = []

        for pos in neighborhood:
            # Check positive potential
            potential = self.maps.get_potential_at_level(pos, 0)
            if potential > 0:
                # Check directly reachable (Definition 3.2)
                if self._is_directly_reachable(current_pos, pos):
                    computing_set.append(pos)

        return computing_set

    def _form_computing_set_Dl(self, neighborhood, level):
        """
        Form computing set Dl theo equation (4) từ paper:
        Dl = {αl ∈ Nl(λ) : Eαl > 0} if l ≥ 1
        """
        computing_set = []

        for pos in neighborhood:
            potential = self.maps.get_potential_at_level(pos, level)
            if potential > 0:
                computing_set.append(pos)

        return computing_set

    def _is_directly_reachable(self, from_pos, to_pos):
        """
        Definition 3.2: Directly Reachable Set
        "An ε-cell is called directly reachable from λ if the line segment joining
        the centroids of λ and that cell is not obstructed by any obstacle cell"

        Simplified implementation: check if to_pos is not obstacle
        """
        if to_pos == from_pos:
            return True

        # Check if target position is obstacle
        row, col = to_pos
        if not self.maps._is_valid_position(row, col):
            return False

        state = self.maps.symbolic_states[row, col]
        return state != 'O'

    def _get_argmax_potential(self, cell_list, level):
        """
        Get cells with maximum potential at given level
        """
        if not cell_list:
            return []

        max_potential = float('-inf')
        max_cells = []

        for pos in cell_list:
            potential = self.maps.get_potential_at_level(pos, level)
            if potential > max_potential:
                max_potential = potential
                max_cells = [pos]
            elif potential == max_potential:
                max_cells.append(pos)

        return max_cells

    def _update_obstacle_information(self, obstacle_locations):
        """
        Update MAPS với newly discovered obstacle locations
        ol ⊂ {1,...|T|}: Vector of obstacle locations
        """
        for pos in obstacle_locations:
            self.maps.set_obstacle(pos)

        # Update MAPS at all levels
        self.maps.update_all_levels()

    def set_environment(self, environment_array):
        """
        Set initial environment from array (1=obstacle, 0=free)
        """
        for row in range(len(environment_array)):
            for col in range(len(environment_array[0])):
                if environment_array[row, col] == 1:
                    self.maps.set_obstacle((row, col))

        # Update MAPS
        self.maps.update_all_levels()

    def is_coverage_complete(self):
        """
        Check if coverage is complete (state = FN)
        """
        return self.state == ETMStates.FN

    def get_current_state(self):
        """Get current ETM state"""
        return self.state