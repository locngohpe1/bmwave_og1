"""
epsilon_star/robot.py
Main Robot Class với Energy Constraints
ε⋆+ Algorithm implementation
"""

import math
import heapq
from typing import Dict, List, Tuple, Optional
from .core import Position, EnergyConfig, SegmentType
from .etm import ExploratoryTuringMachine


class VisibilityGraph:
    """Visibility graph for A* pathfinding"""

    def __init__(self, environment: Dict[Tuple[int, int], str]):
        self.environment = environment

    def neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighbors for A* search"""
        neighbors = []
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        for dr, dc in directions:
            new_pos = (pos[0] + dr, pos[1] + dc)
            if (new_pos in self.environment and
                    self.environment[new_pos] not in ('o', 'obstacle')):
                neighbors.append(new_pos)

        return neighbors

    def weight(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Edge weight (Euclidean distance)"""
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)


def a_star_search(graph: VisibilityGraph, start: Tuple[int, int],
                  goal: Tuple[int, int]) -> Tuple[List[Tuple[int, int]], float]:
    """A* search algorithm"""

    def heuristic(pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

    open_set = [(heuristic(start, goal), start)]
    came_from = {}
    g_score = {start: 0}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            # Reconstruct path
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
    ε⋆+ Algorithm Robot
    Main class integrating ETM với energy constraints
    """

    def __init__(self, battery_pos: Position, rows: int, cols: int,
                 energy_config: EnergyConfig = None):

        self.etm = ExploratoryTuringMachine(rows, cols)
        self.energy_config = energy_config or EnergyConfig()

        # Robot state
        self.current_pos = battery_pos
        self.battery_pos = battery_pos
        self.energy = self.energy_config.capacity

        # Environment for pathfinding
        self.environment = {}

        # Statistics
        self.stats = {
            'total_path_length': 0.0,
            'coverage_length': 0.0,
            'retreat_length': 0.0,
            'advance_length': 0.0,
            'return_count': 0,
            'task_count': 0
        }

        # Current segment
        self.current_segment = SegmentType.COVERAGE

    def set_environment(self, environment_array):
        """Set environment from array (1=obstacle, 0=free)"""
        self.etm.set_environment(environment_array)

        # Create environment dict for A*
        self.environment = {}
        for row in range(len(environment_array)):
            for col in range(len(environment_array[0])):
                pos_tuple = (row, col)
                if environment_array[row, col] == 0:
                    self.environment[pos_tuple] = 'u'  # unvisited
                else:
                    self.environment[pos_tuple] = 'o'  # obstacle

    def run_step(self) -> Dict:
        """Execute one step of ε⋆+ algorithm"""
        if self.etm.is_coverage_complete():
            return {'status': 'complete', 'action': 'none'}

        # Get waypoint from ETM
        waypoints = self.etm.get_waypoint(self.current_pos)

        if not waypoints:
            return {'status': 'no_waypoint', 'action': 'none'}

        # Select best waypoint
        selected_wp = self._select_waypoint(waypoints)

        if selected_wp == self.current_pos:
            # Task at current position
            return self._perform_task()
        else:
            # Move to waypoint
            if self.current_segment == SegmentType.COVERAGE:
                if not self._check_energy_constraint(selected_wp):
                    return self._execute_energy_cycle()

            return self._move_to(selected_wp)

    def _check_energy_constraint(self, next_pos: Position) -> bool:
        """Energy constraint checking using A* on visibility graph"""
        try:
            # Calculate energy for next move
            move_energy = self._calculate_move_energy(next_pos)

            # Calculate return path energy
            retreat_path, retreat_distance = self._calculate_return_path(next_pos)
            retreat_energy = self._calculate_retreat_energy(retreat_distance)

            total_needed = move_energy + retreat_energy
            return self.energy >= total_needed

        except Exception:
            return False

    def _calculate_return_path(self, from_pos: Position) -> Tuple[List[Position], float]:
        """Calculate return path using A* on visibility graph"""
        graph = VisibilityGraph(self.environment)

        try:
            path_tuples, distance = a_star_search(
                graph, from_pos.tuple, self.battery_pos.tuple
            )
            path = [Position(row, col) for row, col in path_tuples]
            return path, distance

        except Exception:
            # Fallback: direct distance
            direct_dist = from_pos.distance_to(self.battery_pos)
            return [from_pos, self.battery_pos], direct_dist

    def _calculate_move_energy(self, to_pos: Position) -> float:
        """Calculate energy for move based on current segment"""
        distance = self.current_pos.distance_to(to_pos)

        if self.current_segment == SegmentType.COVERAGE:
            return self.energy_config.coverage_rate * distance
        elif self.current_segment == SegmentType.ADVANCE:
            return self.energy_config.advance_rate * distance
        elif self.current_segment == SegmentType.RETREAT:
            return self.energy_config.retreat_rate * distance

        return distance

    def _calculate_retreat_energy(self, distance: float) -> float:
        """Calculate energy for retreat segment"""
        return self.energy_config.retreat_rate * distance

    def _execute_energy_cycle(self) -> Dict:
        """Execute retreat-charge-advance cycle"""
        self.stats['return_count'] += 1

        # Execute retreat
        retreat_info = self._execute_retreat()

        # Charge
        self._execute_charge()

        # Execute advance
        advance_info = self._execute_advance()

        return {
            'status': 'energy_cycle_complete',
            'action': 'energy_cycle',
            'retreat_info': retreat_info,
            'advance_info': advance_info
        }

    def _execute_retreat(self) -> Dict:
        """Execute retreat to charging station"""
        self.current_segment = SegmentType.RETREAT

        try:
            retreat_path, distance = self._calculate_return_path(self.current_pos)

            # Follow retreat path
            for next_pos in retreat_path[1:]:  # Skip current position
                self._move_to(next_pos)

            return {'success': True, 'distance': distance}

        except Exception as e:
            return {'success': False, 'error': str(e)}

    def _execute_charge(self):
        """Recharge at charging station"""
        self.energy = self.energy_config.capacity

    def _execute_advance(self) -> Dict:
        """Execute advance to unexplored area"""
        self.current_segment = SegmentType.ADVANCE

        # Find advance target
        target = self._find_advance_target()

        if not target:
            self.current_segment = SegmentType.COVERAGE
            return {'success': False, 'reason': 'no_target'}

        try:
            advance_path, distance = self._calculate_return_path(target)
            advance_path = list(reversed(advance_path))  # Reverse for advance

            # Follow advance path
            for next_pos in advance_path[1:]:  # Skip charging station
                self._move_to(next_pos)

            self.current_segment = SegmentType.COVERAGE
            return {'success': True, 'distance': distance, 'target': target.tuple}

        except Exception as e:
            self.current_segment = SegmentType.COVERAGE
            return {'success': False, 'error': str(e)}

    def _find_advance_target(self) -> Optional[Position]:
        """Find nearby unexplored cell for advance"""
        max_radius = min(30, max(self.etm.maps.rows, self.etm.maps.cols) // 2)

        for radius in range(1, max_radius):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) + abs(dc) != radius:  # Only perimeter
                        continue

                    target = Position(
                        self.battery_pos.row + dr,
                        self.battery_pos.col + dc
                    )

                    target_tuple = target.tuple
                    if (target_tuple in self.environment and
                            self.environment[target_tuple] == 'u'):
                        return target

        return None

    def _select_waypoint(self, waypoints: List[Position]) -> Position:
        """Select waypoint using simplified cost function"""
        if len(waypoints) == 1:
            return waypoints[0]

        # Select waypoint with minimum distance
        return min(waypoints, key=lambda wp: self.current_pos.distance_to(wp))

    def _move_to(self, new_pos: Position) -> Dict:
        """Move robot to new position with energy consumption"""
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

        # Update environment
        pos_tuple = new_pos.tuple
        if pos_tuple in self.environment:
            self.environment[pos_tuple] = 'e'  # explored

        return {
            'status': 'moved',
            'action': 'move',
            'position': new_pos.tuple,
            'segment': self.current_segment.value,
            'energy': self.energy,
            'distance': distance
        }

    def _perform_task(self) -> Dict:
        """Perform task at current position"""
        self.stats['task_count'] += 1

        # Update ETM
        self.etm.task_completed(self.current_pos)

        # Update environment
        pos_tuple = self.current_pos.tuple
        if pos_tuple in self.environment:
            self.environment[pos_tuple] = 'e'

        return {
            'status': 'task_completed',
            'action': 'task',
            'position': self.current_pos.tuple
        }

    def get_statistics(self) -> Dict:
        """Get comprehensive statistics"""
        stats = self.stats.copy()

        # Calculate additional metrics
        total_free_cells = sum(1 for state in self.environment.values()
                               if state in ['e', 'u'])
        explored_cells = sum(1 for state in self.environment.values()
                             if state == 'e')

        if explored_cells > 0:
            # Approximate overlap rate
            estimated_traversed = self.stats['coverage_length']
            overlap_rate = max(0, (estimated_traversed - explored_cells) / explored_cells * 100)
            stats['overlap_rate'] = overlap_rate

        stats['total_free_cells'] = total_free_cells
        stats['explored_cells'] = explored_cells
        stats['coverage_percentage'] = (explored_cells / total_free_cells * 100) if total_free_cells > 0 else 0

        return stats

    def is_complete(self) -> bool:
        """Check if coverage is complete"""
        return self.etm.is_coverage_complete()