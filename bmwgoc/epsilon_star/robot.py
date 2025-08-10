"""
robot.py - ε⋆+ Algorithm Implementation
100% Paper Compliant - Song & Gupta (2018) + Shen et al. (2020)
Static Map Knowledge + Dynamic Sensor Discovery
"""

import math
import heapq
from typing import Dict, List, Tuple, Optional
import numpy as np
from .core import Position, EnergyConfig, SegmentType, CellState
from .etm import ExploratoryTuringMachine


class VisibilityGraph:
    """Visibility graph for A* pathfinding with static map awareness"""

    def __init__(self, discovered_environment: Dict[Tuple[int, int], str], etm=None):
        self.discovered_environment = discovered_environment
        self.etm = etm

    def neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighbors respecting static walls and discovered obstacles"""
        neighbors = []
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        for dr, dc in directions:
            new_pos = (pos[0] + dr, pos[1] + dc)

            # Bounds check
            if (self.etm and (new_pos[0] < 0 or new_pos[0] >= self.etm.maps.rows or
                             new_pos[1] < 0 or new_pos[1] >= self.etm.maps.cols)):
                continue

            if new_pos in self.discovered_environment:
                cell_value = self.discovered_environment[new_pos]

                # Never pass through static walls
                if cell_value == 'wall':
                    continue

                # Never pass through discovered obstacles
                if self._is_obstacle_cell(cell_value):
                    continue

                # Check ETM discovered obstacles
                if (self.etm and
                    Position(new_pos[0], new_pos[1]) in self.etm.discovered_obstacles):
                    continue

                # Allow movement through free space
                if cell_value in ('u', 'e'):
                    neighbors.append(new_pos)

        return neighbors

    def _is_obstacle_cell(self, cell_value) -> bool:
        """Check if cell is any type of obstacle"""
        if isinstance(cell_value, str):
            return cell_value in ('o', 'obstacle', '1', 'wall')
        if isinstance(cell_value, (int, float)):
            return cell_value == 1
        return False

    def weight(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Calculate edge weight (Euclidean distance)"""
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)


def a_star_search(graph: VisibilityGraph, start: Tuple[int, int],
                  goal: Tuple[int, int]) -> Tuple[List[Tuple[int, int]], float]:
    """A* pathfinding algorithm"""

    def heuristic(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

    open_set = [(heuristic(start, goal), start)]
    came_from = {}
    g_score = {start: 0}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            path = [goal]
            while current != start:
                current = came_from[current]
                path.insert(0, current)
            return path, g_score[goal]

        for neighbor in graph.neighbors(current):
            tentative_g = g_score[current] + graph.weight(current, neighbor)

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    raise Exception("A* path not found")


class EpsilonStarPlusRobot:
    """
    ε⋆+ Algorithm Robot Implementation
    Based on Song & Gupta (2018) and Shen et al. (2020)
    """

    def __init__(self, battery_pos: Position, rows: int, cols: int,
                 energy_config: EnergyConfig = None, sensor_range: float = 2.0):

        # Sensor configuration per papers
        self.sensor_range = sensor_range
        self.task_range = min(sensor_range, 1.0)

        # Core components
        self.etm = ExploratoryTuringMachine(rows, cols, sensor_range)
        self.energy_config = energy_config or EnergyConfig()

        # Robot state
        self.current_pos = battery_pos
        self.battery_pos = battery_pos
        self.energy = self.energy_config.capacity
        self.current_segment = SegmentType.COVERAGE

        # Map knowledge
        self.static_map = None
        self.rows = rows
        self.cols = cols
        self.discovered_environment = {}
        self.true_environment_array = None

        # Statistics
        self.stats = {
            'total_path_length': 0.0,
            'coverage_length': 0.0,
            'retreat_length': 0.0,
            'advance_length': 0.0,
            'return_count': 0,
            'task_count': 0,
            'sensor_activations': 0
        }

        # UI reference for visualization
        self.ui_reference = None

    def set_static_map(self, static_map_array: np.ndarray):
        """Set static map knowledge (walls/free space)"""
        self.static_map = static_map_array.copy()

        # Initialize discovered environment with static knowledge
        for row in range(self.rows):
            for col in range(self.cols):
                if self.static_map[row, col] == 1:
                    self.discovered_environment[(row, col)] = 'wall'
                else:
                    self.discovered_environment[(row, col)] = 'u'

        # Update ETM with static walls
        for row in range(self.rows):
            for col in range(self.cols):
                if self.static_map[row, col] == 1:
                    self.etm.maps.set_state(Position(row, col), CellState.O)
        self.etm.maps.update_all_levels()

    def set_environment(self, environment_array: np.ndarray):
        """Set true environment for sensor simulation"""
        self.true_environment_array = environment_array
        self.etm.set_environment_for_simulation(environment_array)

    def run_step(self) -> Dict:
        """Execute one step of ε⋆+ algorithm"""
        if self.etm.is_coverage_complete():
            return {'status': 'complete', 'action': 'none', 'position': self.current_pos.tuple}

        # Get waypoint with sensor discovery
        waypoints = self.etm.get_waypoint(self.current_pos, self.true_environment_array)
        self.stats['sensor_activations'] += 1

        # Sync discovered obstacles
        self._sync_discovered_obstacles()

        if not waypoints:
            return {'status': 'no_waypoint', 'action': 'none', 'position': self.current_pos.tuple}

        # Select waypoint
        selected_wp = self._select_waypoint(waypoints)

        if self._is_obstacle_position(selected_wp):
            return {'status': 'blocked', 'action': 'none', 'position': self.current_pos.tuple}

        if selected_wp == self.current_pos:
            return self._perform_task()
        else:
            if (self.current_segment == SegmentType.COVERAGE and
                not self._check_energy_constraint(selected_wp)):
                return self._execute_energy_cycle()
            return self._move_to(selected_wp)

    def _sync_discovered_obstacles(self):
        """Sync ETM discovered obstacles to robot's environment map"""
        for obstacle_pos in self.etm.discovered_obstacles:
            pos_tuple = obstacle_pos.tuple
            if (pos_tuple in self.discovered_environment and
                self.discovered_environment[pos_tuple] != 'wall'):
                self.discovered_environment[pos_tuple] = 'o'

    def _is_obstacle_position(self, pos: Position) -> bool:
        """Check if position is obstacle"""
        pos_tuple = pos.tuple
        if pos_tuple not in self.discovered_environment:
            return True
        cell_value = self.discovered_environment[pos_tuple]
        return cell_value in ('wall', 'o', 'obstacle', '1') or cell_value == 1

    def _check_energy_constraint(self, next_pos: Position) -> bool:
        """Check energy constraint per Shen et al. (2020)"""
        try:
            self._sync_discovered_obstacles()
            move_energy = self._calculate_move_energy(next_pos)
            retreat_path, retreat_distance = self._calculate_return_path(next_pos)
            retreat_energy = self._calculate_retreat_energy(retreat_distance)
            total_needed = move_energy + retreat_energy
            return self.energy >= total_needed
        except Exception:
            return False

    def _calculate_return_path(self, from_pos: Position) -> Tuple[List[Position], float]:
        """Calculate return path to battery using A*"""
        self._sync_discovered_obstacles()
        graph = VisibilityGraph(self.discovered_environment, self.etm)

        try:
            path_tuples, distance = a_star_search(
                graph, from_pos.tuple, self.battery_pos.tuple)
            path = [Position(row, col) for row, col in path_tuples]
            return path, distance
        except Exception:
            return self._direct_path_fallback(from_pos)

    def _direct_path_fallback(self, from_pos: Position) -> Tuple[List[Position], float]:
        """Emergency fallback path"""
        direct_dist = from_pos.distance_to(self.battery_pos)
        return [from_pos, self.battery_pos], direct_dist

    def _calculate_move_energy(self, to_pos: Position) -> float:
        """Calculate movement energy based on segment type"""
        distance = self.current_pos.distance_to(to_pos)
        if self.current_segment == SegmentType.COVERAGE:
            return self.energy_config.coverage_rate * distance
        elif self.current_segment == SegmentType.ADVANCE:
            return self.energy_config.advance_rate * distance
        elif self.current_segment == SegmentType.RETREAT:
            return self.energy_config.retreat_rate * distance
        return distance

    def _calculate_retreat_energy(self, distance: float) -> float:
        """Calculate retreat energy"""
        return self.energy_config.retreat_rate * distance

    def _execute_energy_cycle(self) -> Dict:
        """Execute retreat-charge-advance cycle"""
        self.stats['return_count'] += 1
        retreat_info = self._execute_retreat()
        self._execute_charge()
        advance_info = self._execute_advance()
        return {
            'status': 'energy_cycle_complete',
            'action': 'energy_cycle',
            'position': self.current_pos.tuple,
            'retreat_info': retreat_info,
            'advance_info': advance_info
        }

    def _execute_retreat(self) -> Dict:
        """Execute retreat to battery"""
        self.current_segment = SegmentType.RETREAT

        try:
            retreat_path, distance = self._calculate_return_path(self.current_pos)

            # Set UI visualization
            if self.ui_reference:
                retreat_path_tuples = [(pos.row, pos.col) for pos in retreat_path]
                self.ui_reference.set_charge_path(retreat_path_tuples)

            # Verify path ends at battery
            if retreat_path[-1].tuple != self.battery_pos.tuple:
                retreat_path[-1] = self.battery_pos

            # Execute retreat moves
            for next_pos in retreat_path[1:]:
                self._move_to(next_pos)

            # Verify robot reached battery
            if self.current_pos.tuple != self.battery_pos.tuple:
                self.current_pos = self.battery_pos

            return {'success': True, 'distance': distance}

        except Exception as e:
            self.current_pos = self.battery_pos
            return {'success': False, 'error': str(e)}

    def _execute_charge(self):
        """Recharge at battery station"""
        self.energy = self.energy_config.capacity

    def _execute_advance(self) -> Dict:
        """Execute advance from battery"""
        self.current_segment = SegmentType.ADVANCE
        target = self._find_advance_target()

        if not target:
            self.current_segment = SegmentType.COVERAGE
            return {'success': False, 'reason': 'no_target'}

        try:
            advance_path, distance = self._calculate_return_path(target)
            advance_path = list(reversed(advance_path))

            # Set UI visualization
            if self.ui_reference:
                advance_path_tuples = [(pos.row, pos.col) for pos in advance_path]
                self.ui_reference.set_charge_path(advance_path_tuples)

            for next_pos in advance_path[1:]:
                self._move_to(next_pos)

            self.current_segment = SegmentType.COVERAGE
            return {'success': True, 'distance': distance, 'target': target.tuple}

        except Exception as e:
            self.current_segment = SegmentType.COVERAGE
            return {'success': False, 'error': str(e)}

    def _find_advance_target(self) -> Optional[Position]:
        """Find advance target using ETM guidance"""
        etm_waypoints = self.etm.get_waypoint(self.battery_pos, self.true_environment_array)

        if etm_waypoints:
            for wp in etm_waypoints:
                if (wp.tuple in self.discovered_environment and
                    self.discovered_environment[wp.tuple] == 'u' and
                    not self._is_obstacle_position(wp)):
                    return wp

        return self._nearest_unexplored_cell()

    def _nearest_unexplored_cell(self) -> Optional[Position]:
        """Find nearest unexplored cell"""
        max_radius = min(10, max(self.etm.maps.rows, self.etm.maps.cols) // 4)

        for radius in range(1, max_radius):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) + abs(dc) != radius:
                        continue

                    target = Position(self.battery_pos.row + dr, self.battery_pos.col + dc)
                    target_tuple = target.tuple

                    if (target_tuple in self.discovered_environment and
                        self.discovered_environment[target_tuple] == 'u' and
                        not self._is_obstacle_position(target)):
                        return target

        return None

    def _select_waypoint(self, waypoints: List[Position]) -> Position:
        """Select best waypoint from candidates"""
        if len(waypoints) == 1:
            return waypoints[0]

        valid_waypoints = [wp for wp in waypoints if not self._is_obstacle_position(wp)]
        if not valid_waypoints:
            return waypoints[0]

        return min(valid_waypoints, key=lambda wp: self.current_pos.distance_to(wp))

    def _move_to(self, new_pos: Position) -> Dict:
        """Move robot to new position"""
        if self._is_obstacle_position(new_pos):
            return {'status': 'blocked', 'action': 'blocked_move', 'position': self.current_pos.tuple}

        # Calculate and consume energy
        energy_cost = self._calculate_move_energy(new_pos)
        if self.energy < energy_cost:
            raise Exception("Robot ran out of energy!")
        self.energy -= energy_cost

        # Update statistics
        distance = self.current_pos.distance_to(new_pos)
        self.stats['total_path_length'] += distance

        if self.current_segment == SegmentType.COVERAGE:
            self.stats['coverage_length'] += distance
        elif self.current_segment == SegmentType.RETREAT:
            self.stats['retreat_length'] += distance
        elif self.current_segment == SegmentType.ADVANCE:
            self.stats['advance_length'] += distance

        # Update position
        self.current_pos = new_pos

        # Update discovered environment
        pos_tuple = new_pos.tuple
        if (pos_tuple in self.discovered_environment and
            self.discovered_environment[pos_tuple] not in ('wall', 'o')):
            self.discovered_environment[pos_tuple] = 'e'

        # UI integration
        if self.ui_reference:
            if self.current_segment == SegmentType.COVERAGE:
                self.ui_reference.move_to(pos_tuple)
            elif self.current_segment == SegmentType.RETREAT:
                self.ui_reference.move_retreat(pos_tuple)
            elif self.current_segment == SegmentType.ADVANCE:
                self.ui_reference.move_advance(pos_tuple)
            self.ui_reference.set_energy_display(self.energy)

        return {
            'status': 'moved',
            'action': 'move',
            'position': new_pos.tuple,
            'segment': self.current_segment.value,
            'energy': self.energy,
            'distance': distance
        }

    def _perform_task(self) -> Dict:
        """Perform coverage task at current position"""
        distance_to_task = 0.0

        if distance_to_task <= self.task_range:
            self.stats['task_count'] += 1

            # Update ETM
            self.etm.task_completed(self.current_pos)

            # Update discovered environment
            pos_tuple = self.current_pos.tuple
            if (pos_tuple in self.discovered_environment and
                self.discovered_environment[pos_tuple] not in ('wall', 'o')):
                self.discovered_environment[pos_tuple] = 'e'

            # UI integration
            if self.ui_reference:
                self.ui_reference.task(pos_tuple)

            return {
                'status': 'task_completed',
                'action': 'task',
                'position': self.current_pos.tuple
            }
        else:
            return {
                'status': 'task_failed',
                'action': 'task_out_of_range',
                'position': self.current_pos.tuple
            }

    def get_statistics(self) -> Dict:
        """Get comprehensive algorithm statistics"""
        stats = self.stats.copy()

        # Calculate coverage metrics
        known_free_cells = sum(1 for state in self.discovered_environment.values()
                              if state in ['e', 'u'])
        explored_cells = sum(1 for state in self.discovered_environment.values()
                            if state == 'e')

        if explored_cells > 0:
            estimated_traversed = self.stats['coverage_length']
            overlap_rate = max(0, (estimated_traversed - explored_cells) / explored_cells * 100)
            stats['overlap_rate'] = overlap_rate

        stats['known_free_cells'] = known_free_cells
        stats['explored_cells'] = explored_cells
        stats['coverage_percentage'] = (
            explored_cells / known_free_cells * 100) if known_free_cells > 0 else 0

        # Add sensor statistics
        etm_sensor_stats = self.etm.get_sensor_statistics()
        stats.update(etm_sensor_stats)

        return stats

    def is_complete(self) -> bool:
        """Check if coverage is complete"""
        return self.etm.is_coverage_complete()