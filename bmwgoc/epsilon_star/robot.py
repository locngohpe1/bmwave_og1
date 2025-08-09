"""
epsilon_star/robot.py - FIXED VERSION
Main Robot Class với PROPER obstacle avoidance
"""

import math
import heapq
from typing import Dict, List, Tuple, Optional
from .core import Position, EnergyConfig, SegmentType
from .etm import ExploratoryTuringMachine


class VisibilityGraph:
    """FIXED Visibility graph for A* pathfinding với proper obstacle handling"""

    def __init__(self, environment: Dict[Tuple[int, int], str]):
        self.environment = environment

    def neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        ✅ FIXED: Get valid neighbors for A* search với proper obstacle checking
        """
        neighbors = []
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        for dr, dc in directions:
            new_pos = (pos[0] + dr, pos[1] + dc)
            if new_pos in self.environment:
                cell_value = self.environment[new_pos]
                # ✅ FIXED: Proper obstacle checking - handle both string and int
                if not self._is_obstacle_cell(cell_value):
                    neighbors.append(new_pos)

        return neighbors

    def _is_obstacle_cell(self, cell_value) -> bool:
        """
        ✅ CRITICAL: Check if cell is obstacle
        Handle various obstacle representations
        """
        # Handle string representations
        if isinstance(cell_value, str):
            return cell_value in ('o', 'obstacle', '1')

        # Handle numeric representations
        if isinstance(cell_value, (int, float)):
            return cell_value == 1

        return False

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
    ✅ FIXED ε⋆+ Algorithm Robot với proper obstacle avoidance
    """

    def __init__(self, battery_pos: Position, rows: int, cols: int,
                 energy_config: EnergyConfig = None):

        self.etm = ExploratoryTuringMachine(rows, cols)
        self.energy_config = energy_config or EnergyConfig()

        # Robot state
        self.current_pos = battery_pos
        self.battery_pos = battery_pos
        self.energy = self.energy_config.capacity

        # ✅ FIXED: Environment for pathfinding với proper obstacle tracking
        self.environment = {}
        self.environment_array = None

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
        """
        ✅ FIXED: Set environment from array with PROPER obstacle handling
        """
        self.environment_array = environment_array
        self.etm.set_environment(environment_array)

        # ✅ FIXED: Create environment dict for A* với proper obstacle marking
        self.environment = {}
        obstacle_count = 0

        for row in range(len(environment_array)):
            for col in range(len(environment_array[0])):
                pos_tuple = (row, col)
                cell_value = environment_array[row, col]

                # ✅ CRITICAL: Handle both 1 and '1' as obstacles
                if cell_value == 1 or cell_value == '1':
                    self.environment[pos_tuple] = 'o'  # obstacle
                    obstacle_count += 1
                else:
                    self.environment[pos_tuple] = 'u'  # unvisited

        total_cells = len(environment_array) * len(environment_array[0])
        free_cells = total_cells - obstacle_count

        print(f"🤖 Robot Environment loaded: {total_cells} total, {obstacle_count} obstacles, {free_cells} free")

    def run_step(self) -> Dict:
        """Execute one step of ε⋆+ algorithm"""
        if self.etm.is_coverage_complete():
            return {'status': 'complete', 'action': 'none'}

        # Get waypoint from ETM
        waypoints = self.etm.get_waypoint(self.current_pos)

        if not waypoints:
            return {'status': 'no_waypoint', 'action': 'none'}

        # ✅ FIXED: Select best waypoint với obstacle validation
        selected_wp = self._select_waypoint(waypoints)

        # ✅ CRITICAL: Validate waypoint is not obstacle before moving
        if self._is_obstacle_position(selected_wp):
            print(f"⚠️  WARNING: ETM suggested obstacle waypoint {selected_wp.tuple}! Searching alternative...")
            alternative_wp = self._find_alternative_waypoint(waypoints)
            if alternative_wp:
                selected_wp = alternative_wp
                print(f"✅ Found alternative waypoint: {selected_wp.tuple}")
            else:
                print(f"❌ No valid alternative waypoint found!")
                return {'status': 'blocked', 'action': 'none'}

        if selected_wp == self.current_pos:
            # Task at current position
            return self._perform_task()
        else:
            # Move to waypoint
            if self.current_segment == SegmentType.COVERAGE:
                if not self._check_energy_constraint(selected_wp):
                    return self._execute_energy_cycle()

            return self._move_to(selected_wp)

    def _is_obstacle_position(self, pos: Position) -> bool:
        """
        ✅ CRITICAL: Check if position is obstacle
        This prevents robot from moving into walls!
        """
        pos_tuple = pos.tuple
        if pos_tuple not in self.environment:
            return True  # Out of bounds = obstacle

        cell_value = self.environment[pos_tuple]
        return cell_value in ('o', 'obstacle', '1') or cell_value == 1

    def _find_alternative_waypoint(self, waypoints: List[Position]) -> Optional[Position]:
        """Find non-obstacle waypoint from list"""
        for wp in waypoints:
            if not self._is_obstacle_position(wp):
                return wp
        return None

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

        except Exception as e:
            print(f"⚠️  Energy constraint check failed: {e}")
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

        except Exception as e:
            print(f"⚠️  A* pathfinding failed: {e}, using direct distance")
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
                            self.environment[target_tuple] == 'u' and
                            not self._is_obstacle_position(target)):
                        return target

        return None

    def _select_waypoint(self, waypoints: List[Position]) -> Position:
        """Select waypoint using simplified cost function"""
        if len(waypoints) == 1:
            return waypoints[0]

        # ✅ FIXED: Filter out obstacles first
        valid_waypoints = [wp for wp in waypoints if not self._is_obstacle_position(wp)]

        if not valid_waypoints:
            print(f"⚠️  WARNING: All waypoints are obstacles!")
            return waypoints[0]  # Return first one anyway, will be caught later

        # Select waypoint with minimum distance
        return min(valid_waypoints, key=lambda wp: self.current_pos.distance_to(wp))

    def _move_to(self, new_pos: Position) -> Dict:
        """
        ✅ FIXED: Move robot to new position với obstacle validation
        """
        # ✅ CRITICAL: Final obstacle check before moving
        if self._is_obstacle_position(new_pos):
            print(f"🚫 BLOCKED: Cannot move to obstacle at {new_pos.tuple}")
            return {
                'status': 'blocked',
                'action': 'blocked_move',
                'position': new_pos.tuple,
                'reason': 'obstacle'
            }

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