"""
epsilon_star/core.py
Core classes và data structures cho ε⋆+ Algorithm
"""

import numpy as np
import math
from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple, Optional


class ETMState(Enum):
    """ETM States theo Definition 3.1"""
    ST = "START"
    CP0 = "COMPUTE_L0"
    CP1 = "COMPUTE_L1"
    CP2 = "COMPUTE_L2"
    WT = "WAIT"
    FN = "FINISH"


class CellState(Enum):
    """Symbolic states cho ε-cells"""
    O = "OBSTACLE"
    F = "FORBIDDEN"
    E = "EXPLORED"
    U = "UNEXPLORED"


class SegmentType(Enum):
    """Trajectory segment types"""
    COVERAGE = "coverage"
    ADVANCE = "advance"
    RETREAT = "retreat"


@dataclass
class Position:
    """Position với helper methods"""
    row: int
    col: int

    @property
    def tuple(self) -> Tuple[int, int]:
        return (self.row, self.col)

    def distance_to(self, other: 'Position') -> float:
        return math.sqrt((self.row - other.row) ** 2 + (self.col - other.col) ** 2)

    def neighbors_8(self) -> List['Position']:
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        return [Position(self.row + dr, self.col + dc) for dr, dc in directions]

    def __hash__(self):
        return hash((self.row, self.col))

    def __eq__(self, other):
        return isinstance(other, Position) and self.row == other.row and self.col == other.col


@dataclass
class EnergyConfig:
    """Energy configuration theo paper"""
    capacity: float = 1000.0
    coverage_rate: float = 2.0  # "twice this amount for coverage segment"
    advance_rate: float = 1.0  # "proportional to trajectory length"
    retreat_rate: float = 1.0


class MAPSHierarchy:
    """
    Multiscale Adaptive Potential Surfaces
    Implementation theo Section 3-A từ paper
    """

    def __init__(self, rows: int, cols: int):
        self.rows = rows
        self.cols = cols

        # Build hierarchical tiling
        self.tilings = self._build_hierarchical_tiling()
        self.max_level = len(self.tilings) - 1

        # Potential surfaces for each level
        self.potential_surfaces = [np.zeros(shape) for shape in self.tilings]

        # Exogenous potential field B
        self.field_B = self._create_field_B()

        # Symbolic states (Level 0 only)
        self.states = np.full((rows, cols), CellState.U, dtype=object)

    def _build_hierarchical_tiling(self) -> List[Tuple[int, int]]:
        """Build MST theo recursive decomposition"""
        tilings = [(self.rows, self.cols)]
        current_rows, current_cols = self.rows, self.cols

        while True:
            # Divide rows
            if current_rows % 2 == 0:
                new_rows = current_rows // 2
            else:
                n_prime = (current_rows + 1) // 2
                if n_prime - 1 < 2:
                    break
                new_rows = n_prime

            # Divide cols
            if current_cols % 2 == 0:
                new_cols = current_cols // 2
            else:
                n_prime = (current_cols + 1) // 2
                if n_prime - 1 < 2:
                    break
                new_cols = n_prime

            if new_rows < 2 or new_cols < 2:
                break

            tilings.append((new_rows, new_cols))
            current_rows, current_cols = new_rows, new_cols

        return tilings

    def _create_field_B(self) -> np.ndarray:
        """Create exogenous potential field B"""
        B = np.zeros((self.rows, self.cols))
        B_max = self.cols

        for row in range(self.rows):
            for col in range(self.cols):
                B[row, col] = B_max - col  # Decreasing left to right

        return B

    def update_level_0(self):
        """Update potential surface Level 0 theo Equation (2)"""
        E0 = self.potential_surfaces[0]

        for row in range(self.rows):
            for col in range(self.cols):
                state = self.states[row, col]

                if state in (CellState.O, CellState.F):
                    E0[row, col] = -1
                elif state == CellState.E:
                    E0[row, col] = 0
                elif state == CellState.U:
                    E0[row, col] = self.field_B[row, col]

    def update_level_l(self, level: int):
        """Update potential surface Level l theo Equation (3)"""
        if level == 0:
            self.update_level_0()
            return

        if level > self.max_level:
            return

        El = self.potential_surfaces[level]
        level_rows, level_cols = self.tilings[level]

        for coarse_row in range(level_rows):
            for coarse_col in range(level_cols):
                epsilon_cells = self._get_epsilon_cells_in_coarse_cell(
                    coarse_row, coarse_col, level
                )

                if not epsilon_cells:
                    El[coarse_row, coarse_col] = 0
                    continue

                # Calculate p^U_αℓ
                total_cells = len(epsilon_cells)
                unexplored_count = sum(
                    1 for pos in epsilon_cells
                    if self.states[pos.row, pos.col] == CellState.U
                )
                p_U = unexplored_count / total_cells if total_cells > 0 else 0

                # Calculate mean B_αℓ
                B_mean = np.mean([self.field_B[pos.row, pos.col] for pos in epsilon_cells])

                # Apply Equation (3)
                El[coarse_row, coarse_col] = p_U * B_mean

    def _get_epsilon_cells_in_coarse_cell(self, coarse_row: int, coarse_col: int,
                                          level: int) -> List[Position]:
        """Get all ε-cells belonging to coarse cell"""
        epsilon_cells = []
        scale_factor = 2 ** level

        start_row = coarse_row * scale_factor
        end_row = min((coarse_row + 1) * scale_factor, self.rows)
        start_col = coarse_col * scale_factor
        end_col = min((coarse_col + 1) * scale_factor, self.cols)

        for row in range(start_row, end_row):
            for col in range(start_col, end_col):
                epsilon_cells.append(Position(row, col))

        return epsilon_cells

    def get_local_neighborhood(self, pos: Position, level: int = 0) -> List[Position]:
        """Get local neighborhood N^ℓ(λ)"""
        if level == 0:
            neighbors = []
            for neighbor_pos in pos.neighbors_8():
                if self._is_valid_position(neighbor_pos, level):
                    neighbors.append(neighbor_pos)
            neighbors.append(pos)
            return neighbors
        else:
            neighbors = []
            level_rows, level_cols = self.tilings[level]

            for neighbor_pos in pos.neighbors_8():
                if (0 <= neighbor_pos.row < level_rows and
                        0 <= neighbor_pos.col < level_cols):
                    neighbors.append(neighbor_pos)
            neighbors.append(pos)
            return neighbors

    def map_to_coarse_level(self, epsilon_pos: Position, level: int) -> Position:
        """Map ε-cell to coarse cell at given level"""
        if level == 0:
            return epsilon_pos

        scale_factor = 2 ** level
        coarse_row = epsilon_pos.row // scale_factor
        coarse_col = epsilon_pos.col // scale_factor
        return Position(coarse_row, coarse_col)

    def select_epsilon_cell_in_coarse_cell(self, coarse_pos: Position,
                                           level: int) -> Position:
        """Function I(·): select unexplored ε-cell in coarse cell"""
        if level == 0:
            return coarse_pos

        epsilon_cells = self._get_epsilon_cells_in_coarse_cell(
            coarse_pos.row, coarse_pos.col, level
        )

        # Filter unexplored cells
        unexplored_cells = [
            pos for pos in epsilon_cells
            if self.states[pos.row, pos.col] == CellState.U
        ]

        if unexplored_cells:
            return unexplored_cells[0]
        elif epsilon_cells:
            return epsilon_cells[0]
        else:
            return coarse_pos

    def get_potential(self, pos: Position, level: int = 0) -> float:
        """Get potential value at position and level"""
        if level == 0:
            if self._is_valid_position(pos, level):
                return self.potential_surfaces[0][pos.row, pos.col]
        else:
            if level <= self.max_level and self._is_valid_position(pos, level):
                return self.potential_surfaces[level][pos.row, pos.col]
        return -1

    def _is_valid_position(self, pos: Position, level: int = 0) -> bool:
        """Check if position is valid for given level"""
        if level == 0:
            return 0 <= pos.row < self.rows and 0 <= pos.col < self.cols
        else:
            if level <= self.max_level:
                level_rows, level_cols = self.tilings[level]
                return 0 <= pos.row < level_rows and 0 <= pos.col < level_cols
        return False

    def set_state(self, pos: Position, state: CellState):
        """Set state of ε-cell"""
        if self._is_valid_position(pos, 0):
            self.states[pos.row, pos.col] = state

    def update_all_levels(self):
        """Update all potential surfaces"""
        for level in range(self.max_level + 1):
            self.update_level_l(level)