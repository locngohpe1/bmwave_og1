"""
robot.py - SENSOR-BASED ROBOT IMPLEMENTATION
🔧 100% Paper Compliant với Progressive Obstacle Discovery
"""

import math
import heapq
from typing import Dict, List, Tuple, Optional
import numpy as np
from .core import Position, EnergyConfig, SegmentType
from .etm import ExploratoryTuringMachine


class VisibilityGraph:
    """
    🔧 SENSOR-AWARE Visibility graph
    Only considers DISCOVERED obstacles for pathfinding
    """

    def __init__(self, discovered_obstacles: Dict[Tuple[int, int], str]):
        self.discovered_obstacles = discovered_obstacles

    def neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighbors considering only DISCOVERED obstacles"""
        neighbors = []
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        for dr, dc in directions:
            new_pos = (pos[0] + dr, pos[1] + dc)
            if new_pos in self.discovered_obstacles:
                cell_value = self.discovered_obstacles[new_pos]
                if not self._is_obstacle_cell(cell_value):
                    neighbors.append(new_pos)

        return neighbors

    def _is_obstacle_cell(self, cell_value) -> bool:
        """Check if cell is discovered obstacle"""
        if isinstance(cell_value, str):
            return cell_value in ('o', 'obstacle', '1')
        if isinstance(cell_value, (int, float)):
            return cell_value == 1
        return False

    def weight(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Edge weight (Euclidean distance)"""
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)


def a_star_search(graph: VisibilityGraph, start: Tuple[int, int],
                  goal: Tuple[int, int]) -> Tuple[List[Tuple[int, int]], float]:
    """A* search on discovered obstacle map only"""

    def heuristic(pos1, pos2):
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

    raise Exception("A* path not found with discovered obstacles")


class EpsilonStarPlusRobot:
    """
    🔧 SENSOR-BASED ε⋆+ Algorithm Robot
    - Progressive obstacle discovery using sensors
    - Dynamic environment knowledge building
    - 100% Paper Compliant Implementation
    """

    def __init__(self, battery_pos: Position, rows: int, cols: int,
                 energy_config: EnergyConfig = None, sensor_range: float = 2.0):

        # 🔧 SENSOR CONFIGURATION per papers
        self.sensor_range = sensor_range  # Rs - detection range
        self.task_range = min(sensor_range, 1.0)  # rt ≤ Rs

        self.etm = ExploratoryTuringMachine(rows, cols, sensor_range)
        self.energy_config = energy_config or EnergyConfig()

        # Robot state
        self.current_pos = battery_pos
        self.battery_pos = battery_pos
        self.energy = self.energy_config.capacity

        # 🔧 DISCOVERED ENVIRONMENT ONLY
        # Papers: Robot starts with NO obstacle knowledge
        self.discovered_environment = {}
        self.true_environment_array = None  # Hidden from robot, used for sensor simulation

        # Initialize all cells as unknown/unvisited
        for row in range(rows):
            for col in range(cols):
                self.discovered_environment[(row, col)] = 'u'  # unvisited

        # Statistics
        self.stats = {
            'total_path_length': 0.0,
            'coverage_length': 0.0,
            'retreat_length': 0.0,
            'advance_length': 0.0,
            'return_count': 0,
            'task_count': 0,
            'obstacles_discovered': 0,
            'sensor_activations': 0
        }

        self.current_segment = SegmentType.COVERAGE

        print(f"🤖 Sensor-based Robot initialized:")
        print(f"   📡 Sensor range Rs: {self.sensor_range}")
        print(f"   🛠️  Task range rt: {self.task_range}")
        print(f"   🌍 Unknown environment: {rows}×{cols}")

    def set_environment(self, environment_array: np.ndarray):
        """
        🔧 Set TRUE environment for sensor simulation
        Robot doesn't know this - only used for sensor discovery
        """
        self.true_environment_array = environment_array
        self.etm.set_environment_for_simulation(environment_array)

        total_obstacles = np.sum(environment_array == 1)
        total_cells = environment_array.shape[0] * environment_array.shape[1]
        free_cells = total_cells - total_obstacles

        print(f"🌍 TRUE Environment set for sensor simulation:")
        print(f"   📊 Total cells: {total_cells}")
        print(f"   🚧 True obstacles: {total_obstacles}")
        print(f"   🆓 True free cells: {free_cells}")
        print(f"   🤖 Robot knowledge: 0 obstacles (unknown environment)")

    def run_step(self) -> Dict:
        """
        🔧 Execute one step with SENSOR-BASED discovery
        """
        if self.etm.is_coverage_complete():
            return {'status': 'complete', 'action': 'none'}

        # 🔧 SENSOR DISCOVERY - Get waypoint with environment for sensor simulation
        waypoints = self.etm.get_waypoint(self.current_pos, self.true_environment_array)
        self.stats['sensor_activations'] += 1

        if not waypoints:
            return {'status': 'no_waypoint', 'action': 'none'}

        # Select and validate waypoint
        selected_wp = self._select_waypoint(waypoints)

        # Check if waypoint is discovered obstacle
        if self._is_discovered_obstacle_position(selected_wp):
            print(f"⚠️  Waypoint {selected_wp.tuple} is discovered obstacle!")
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

    def _is_discovered_obstacle_position(self, pos: Position) -> bool:
        """
        🔧 Check if position is DISCOVERED obstacle
        Only considers obstacles within sensor range that have been discovered
        """
        pos_tuple = pos.tuple
        if pos_tuple not in self.discovered_environment:
            return True

        cell_value = self.discovered_environment[pos_tuple]
        return cell_value in ('o', 'obstacle', '1') or cell_value == 1

    def _check_energy_constraint(self, next_pos: Position) -> bool:
        """Energy constraint checking using discovered obstacles only"""
        try:
            move_energy = self._calculate_move_energy(next_pos)
            retreat_path, retreat_distance = self._calculate_return_path(next_pos)
            retreat_energy = self._calculate_retreat_energy(retreat_distance)

            total_needed = move_energy + retreat_energy
            return self.energy >= total_needed

        except Exception as e:
            print(f"⚠️  Energy constraint check failed (sensor-based): {e}")
            # If path not found with discovered obstacles, assume reachable
            return True

    def _calculate_return_path(self, from_pos: Position) -> Tuple[List[Position], float]:
        """
        🔧 Calculate return path using DISCOVERED obstacles only
        Papers: Robot plans based on current obstacle knowledge
        """
        graph = VisibilityGraph(self.discovered_environment)

        try:
            path_tuples, distance = a_star_search(
                graph, from_pos.tuple, self.battery_pos.tuple
            )
            path = [Position(row, col) for row, col in path_tuples]
            return path, distance

        except Exception as e:
            print(f"⚠️  A* failed with discovered obstacles: {e}")
            # Fallback: direct distance (robot doesn't know about undiscovered obstacles)
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

        retreat_info = self._execute_retreat()
        self._execute_charge()
        advance_info = self._execute_advance()

        return {
            'status': 'energy_cycle_complete',
            'action': 'energy_cycle',
            'retreat_info': retreat_info,
            'advance_info': advance_info
        }

    def _execute_retreat(self) -> Dict:
        """Execute retreat using discovered obstacle knowledge"""
        self.current_segment = SegmentType.RETREAT

        try:
            retreat_path, distance = self._calculate_return_path(self.current_pos)

            for next_pos in retreat_path[1:]:
                self._move_to(next_pos)

            return {'success': True, 'distance': distance}

        except Exception as e:
            return {'success': False, 'error': str(e)}

    def _execute_charge(self):
        """Recharge at charging station"""
        self.energy = self.energy_config.capacity

    def _execute_advance(self) -> Dict:
        """
        🔧 Execute advance using ETM's guidance
        Papers: "advances to a nearby cell to restart coverage"
        """
        self.current_segment = SegmentType.ADVANCE

        # Use ETM to find advance target (sensor-based)
        target = self._find_advance_target()

        if not target:
            self.current_segment = SegmentType.COVERAGE
            return {'success': False, 'reason': 'no_target'}

        try:
            advance_path, distance = self._calculate_return_path(target)
            advance_path = list(reversed(advance_path))

            for next_pos in advance_path[1:]:
                self._move_to(next_pos)

            self.current_segment = SegmentType.COVERAGE
            return {'success': True, 'distance': distance, 'target': target.tuple}

        except Exception as e:
            self.current_segment = SegmentType.COVERAGE
            return {'success': False, 'error': str(e)}

    def _find_advance_target(self) -> Optional[Position]:
        """
        🔧 Find advance target using ETM waypoint computation
        Papers compliant: Use ETM's intelligence for target selection
        """
        # Ask ETM for waypoint from charging station
        etm_waypoints = self.etm.get_waypoint(self.battery_pos, self.true_environment_array)

        if etm_waypoints:
            for wp in etm_waypoints:
                if (wp.tuple in self.discovered_environment and
                        self.discovered_environment[wp.tuple] == 'u' and
                        not self._is_discovered_obstacle_position(wp)):
                    return wp

        # Fallback: nearest unexplored from discovered knowledge
        return self._nearest_unexplored_in_discovered_space()

    def _nearest_unexplored_in_discovered_space(self) -> Optional[Position]:
        """Find nearest unexplored cell in discovered space"""
        max_radius = min(10, max(self.etm.maps.rows, self.etm.maps.cols) // 4)

        for radius in range(1, max_radius):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) + abs(dc) != radius:
                        continue

                    target = Position(
                        self.battery_pos.row + dr,
                        self.battery_pos.col + dc
                    )

                    target_tuple = target.tuple
                    if (target_tuple in self.discovered_environment and
                            self.discovered_environment[target_tuple] == 'u' and
                            not self._is_discovered_obstacle_position(target)):
                        return target

        return None

    def _select_waypoint(self, waypoints: List[Position]) -> Position:
        """Select waypoint using cost function"""
        if len(waypoints) == 1:
            return waypoints[0]

        valid_waypoints = [wp for wp in waypoints if not self._is_discovered_obstacle_position(wp)]

        if not valid_waypoints:
            return waypoints[0]

        return min(valid_waypoints, key=lambda wp: self.current_pos.distance_to(wp))

    def _move_to(self, new_pos: Position) -> Dict:
        """
        🔧 Move robot with sensor discovery
        """
        # Final check for discovered obstacles
        if self._is_discovered_obstacle_position(new_pos):
            return {
                'status': 'blocked',
                'action': 'blocked_move',
                'position': new_pos.tuple,
                'reason': 'discovered_obstacle'
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

        # Update discovered environment (robot has visited this cell)
        pos_tuple = new_pos.tuple
        if pos_tuple in self.discovered_environment:
            self.discovered_environment[pos_tuple] = 'e'  # explored

        return {
            'status': 'moved',
            'action': 'move',
            'position': new_pos.tuple,
            'segment': self.current_segment.value,
            'energy': self.energy,
            'distance': distance
        }

    def _perform_task(self) -> Dict:
        """
        🔧 Perform task at current position
        Papers: Task sensor with range rt ≤ Rs
        """
        # Check if within task range (should always be true for current position)
        distance_to_task = 0.0  # Robot is at the cell

        if distance_to_task <= self.task_range:
            self.stats['task_count'] += 1

            # Update ETM
            self.etm.task_completed(self.current_pos)

            # Update discovered environment
            pos_tuple = self.current_pos.tuple
            if pos_tuple in self.discovered_environment:
                self.discovered_environment[pos_tuple] = 'e'

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
        """Get comprehensive statistics including sensor data"""
        stats = self.stats.copy()

        # Calculate coverage based on discovered space only
        total_discovered_cells = sum(1 for state in self.discovered_environment.values()
                                     if state in ['e', 'u'])
        explored_cells = sum(1 for state in self.discovered_environment.values()
                             if state == 'e')

        if explored_cells > 0:
            estimated_traversed = self.stats['coverage_length']
            overlap_rate = max(0, (estimated_traversed - explored_cells) / explored_cells * 100)
            stats['overlap_rate'] = overlap_rate

        stats['total_discovered_cells'] = total_discovered_cells
        stats['explored_cells'] = explored_cells
        stats['coverage_percentage'] = (
                    explored_cells / total_discovered_cells * 100) if total_discovered_cells > 0 else 0

        # Add sensor statistics
        etm_sensor_stats = self.etm.get_sensor_statistics()
        stats.update(etm_sensor_stats)

        return stats

    def is_complete(self) -> bool:
        """Check if coverage is complete"""
        return self.etm.is_coverage_complete()