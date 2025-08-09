import numpy as np
class MAPSHierarchy:
    """
    Multiscale Adaptive Potential Surfaces theo Section 3-A từ paper
    """

    def __init__(self, row_count, col_count):
        self.row_count = row_count
        self.col_count = col_count

        # Build hierarchical multiscale tiling (MST)
        self.tilings = self._build_hierarchical_tiling()
        self.L = len(self.tilings) - 1  # Maximum level

        # Potential surfaces for each level
        self.potential_surfaces = [np.zeros(tiling_shape) for tiling_shape in self.tilings]

        # Exogenous potential field B
        self.field_B = self._create_field_B()

        # Symbolic states for ε-cells (Level 0 only)
        self.symbolic_states = np.full((row_count, col_count), 'U', dtype=object)

    def _build_hierarchical_tiling(self):
        """
        Build hierarchical multiscale tiling theo paper:
        "recursive decomposition" with "n/2 < 2 or n'-1 < 2" stopping condition
        """
        tilings = []

        # Level 0: ε-cell tiling (finest level)
        tilings.append((self.row_count, self.col_count))

        # Higher levels: recursive decomposition
        current_rows, current_cols = self.row_count, self.col_count

        while True:
            # Check stopping condition from paper
            n_rows, n_cols = current_rows, current_cols

            # Divide along x-axis
            if n_rows % 2 == 0:
                new_rows = n_rows // 2
            else:
                n_prime = (n_rows + 1) // 2
                if n_prime - 1 < 2:
                    break
                new_rows = n_prime

            # Divide along y-axis
            if n_cols % 2 == 0:
                new_cols = n_cols // 2
            else:
                n_prime = (n_cols + 1) // 2
                if n_prime - 1 < 2:
                    break
                new_cols = n_prime

            # Check stopping condition
            if new_rows < 2 or new_cols < 2:
                break

            tilings.append((new_rows, new_cols))
            current_rows, current_cols = new_rows, new_cols

        return tilings

    def _create_field_B(self):
        """
        Create exogenous potential field B theo paper:
        "plateaus of equipotential surfaces along each column of the tiling"
        "monotonically increase in height by one unit from 1 on the rightmost column to Bmax on the leftmost column"
        """
        B = np.zeros((self.row_count, self.col_count))
        Bmax = self.col_count

        for row in range(self.row_count):
            for col in range(self.col_count):
                # Decreasing from left (Bmax) to right (1)
                B[row, col] = Bmax - col

        return B

    def update_potential_surface_level_0(self):
        """
        Update potential surface at Level 0 theo Equation (2):
        Eα0(k) = {-1 if O or F, 0 if E, Bα0 if U}
        """
        E0 = self.potential_surfaces[0]

        for row in range(self.row_count):
            for col in range(self.col_count):
                state = self.symbolic_states[row, col]

                if state in ('O', 'F'):  # Obstacle or Forbidden
                    E0[row, col] = -1
                elif state == 'E':  # Explored
                    E0[row, col] = 0
                elif state == 'U':  # Unexplored
                    E0[row, col] = self.field_B[row, col]

    def update_potential_surface_level_l(self, level):
        """
        Update potential surface at Level l (l ≥ 1) theo Equation (3):
        Eαℓ(k) = pUαℓ(k)Bαℓ
        """
        if level == 0:
            self.update_potential_surface_level_0()
            return

        if level > self.L:
            return

        El = self.potential_surfaces[level]
        level_rows, level_cols = self.tilings[level]

        for coarse_row in range(level_rows):
            for coarse_col in range(level_cols):
                # Get constituent ε-cells within this coarse cell
                epsilon_cells = self._get_epsilon_cells_in_coarse_cell(coarse_row, coarse_col, level)

                if not epsilon_cells:
                    El[coarse_row, coarse_col] = 0
                    continue

                # Calculate probability of unexplored ε-cells
                total_cells = len(epsilon_cells)
                unexplored_count = sum(1 for (r, c) in epsilon_cells
                                       if self.symbolic_states[r, c] == 'U')

                pU = unexplored_count / total_cells if total_cells > 0 else 0

                # Calculate mean exogenous potential Bαℓ
                B_mean = np.mean([self.field_B[r, c] for (r, c) in epsilon_cells])

                # Apply equation (3)
                El[coarse_row, coarse_col] = pU * B_mean

    def _get_epsilon_cells_in_coarse_cell(self, coarse_row, coarse_col, level):
        """
        Get all ε-cells (Level 0) that belong to a coarse cell at given level
        """
        epsilon_cells = []

        # Calculate scaling factor
        scale_factor = 2 ** level

        # Calculate bounds for ε-cells
        start_row = coarse_row * scale_factor
        end_row = min((coarse_row + 1) * scale_factor, self.row_count)
        start_col = coarse_col * scale_factor
        end_col = min((coarse_col + 1) * scale_factor, self.col_count)

        for row in range(start_row, end_row):
            for col in range(start_col, end_col):
                epsilon_cells.append((row, col))

        return epsilon_cells

    def get_local_neighborhood_level_0(self, pos):
        """
        Get local neighborhood N0(λ) at Level 0
        """
        row, col = pos
        neighborhood = []

        # 8-connected neighborhood
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if self._is_valid_position(new_row, new_col):
                neighborhood.append((new_row, new_col))

        # Include current position
        neighborhood.append(pos)

        return neighborhood

    def get_local_neighborhood_level_l(self, coarse_pos, level):
        """
        Get local neighborhood Nl(λ) at Level l (l ≥ 1)
        """
        if level == 0:
            return self.get_local_neighborhood_level_0(coarse_pos)

        coarse_row, coarse_col = coarse_pos
        level_rows, level_cols = self.tilings[level]
        neighborhood = []

        # 8-connected neighborhood at coarse level
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        for dr, dc in directions:
            new_row, new_col = coarse_row + dr, coarse_col + dc
            if 0 <= new_row < level_rows and 0 <= new_col < level_cols:
                neighborhood.append((new_row, new_col))

        # Include current coarse position
        neighborhood.append(coarse_pos)

        return neighborhood

    def map_epsilon_cell_to_coarse_level(self, epsilon_pos, level):
        """
        Map ε-cell position to coarse cell position at given level
        """
        if level == 0:
            return epsilon_pos

        row, col = epsilon_pos
        scale_factor = 2 ** level

        coarse_row = row // scale_factor
        coarse_col = col // scale_factor

        return (coarse_row, coarse_col)

    def select_random_epsilon_cell_in_coarse_cell(self, coarse_pos, level):
        """
        Function I(·) from paper: randomly select unexplored ε-cell in coarse cell
        """
        if level == 0:
            return coarse_pos

        coarse_row, coarse_col = coarse_pos
        epsilon_cells = self._get_epsilon_cells_in_coarse_cell(coarse_row, coarse_col, level)

        # Filter unexplored cells
        unexplored_cells = [(r, c) for (r, c) in epsilon_cells
                            if self.symbolic_states[r, c] == 'U']

        if not unexplored_cells:
            # Return first cell if no unexplored cells
            return epsilon_cells[0] if epsilon_cells else coarse_pos

        # Randomly select (for simplicity, take first)
        return unexplored_cells[0]

    def set_obstacle(self, pos):
        """Set ε-cell as obstacle"""
        row, col = pos
        if self._is_valid_position(row, col):
            self.symbolic_states[row, col] = 'O'

    def set_explored(self, pos):
        """Set ε-cell as explored"""
        row, col = pos
        if self._is_valid_position(row, col):
            self.symbolic_states[row, col] = 'E'

    def set_forbidden(self, pos):
        """Set ε-cell as forbidden"""
        row, col = pos
        if self._is_valid_position(row, col):
            self.symbolic_states[row, col] = 'F'

    def _is_valid_position(self, row, col):
        """Check if position is within ε-cell tiling bounds"""
        return 0 <= row < self.row_count and 0 <= col < self.col_count

    def get_potential_at_level(self, pos, level):
        """Get potential value at given position and level"""
        if level == 0:
            row, col = pos
            if self._is_valid_position(row, col):
                return self.potential_surfaces[0][row, col]
        else:
            if level <= self.L:
                coarse_row, coarse_col = pos
                level_rows, level_cols = self.tilings[level]
                if 0 <= coarse_row < level_rows and 0 <= coarse_col < level_cols:
                    return self.potential_surfaces[level][coarse_row, coarse_col]

        return -1  # Invalid position

    def update_all_levels(self):
        """Update potential surfaces at all levels"""
        for level in range(self.L + 1):
            self.update_potential_surface_level_l(level)