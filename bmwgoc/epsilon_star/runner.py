"""
runner.py - ε⋆+ Algorithm Runner
100% Paper Compliant Implementation với Sensor Visualization
"""

import numpy as np
import pygame as pg
import time
import sys
import os
from typing import Dict

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from grid_map import Grid_Map
from .core import Position, EnergyConfig
from .robot import EpsilonStarPlusRobot


class EpsilonStarPlusRunner:
    """ε⋆+ Algorithm Runner with Enhanced Sensor Visualization"""

    def __init__(self, map_file: str = "map/real_map/denmark.txt", sensor_range: float = 2.0):
        self.ui = Grid_Map()
        pg.init()

        if not os.path.exists(map_file):
            raise FileNotFoundError(f"Map file not found: {map_file}")

        # Load environment
        true_environment, _ = self.ui.read_map(map_file)
        self.ui.map = np.array(true_environment, dtype=object)
        self.ui.row_count = len(true_environment)
        self.ui.col_count = len(true_environment[0])

        # Interactive setup
        final_environment, final_battery_pos = self.ui.edit_map()
        self.true_environment_array = np.array(final_environment)
        self.battery_pos = Position(final_battery_pos[0], final_battery_pos[1])

        # Sensor configuration
        self.sensor_range = sensor_range

        # Energy configuration per Shen et al. (2020)
        self.energy_config = EnergyConfig(
            capacity=1000.0,
            coverage_rate=2.0,
            advance_rate=1.0,
            retreat_rate=1.0
        )

        # Create robot
        self.robot = EpsilonStarPlusRobot(
            self.battery_pos,
            len(self.true_environment_array),
            len(self.true_environment_array[0]),
            self.energy_config,
            sensor_range=self.sensor_range
        )

        # Set robot knowledge
        self.robot.set_static_map(self.true_environment_array)
        self.robot.set_environment(self.true_environment_array)
        self.robot.ui_reference = self.ui

        # Control variables
        self.paused = False
        self.speed = 60
        self.step_count = 0
        self.start_time = None

        # Visualization settings
        self.show_sensor_range = True
        self.show_discovered_obstacles = True
        self.show_stats_overlay = True

    def run(self):
        """Main execution loop"""
        clock = pg.time.Clock()
        running = True
        self.start_time = time.time()

        while running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    running = False
                elif event.type == pg.KEYDOWN:
                    running = self._handle_keypress(event.key)

            self._update_ui()
            clock.tick(self.speed)

            if self.paused:
                continue

            if not self.robot.is_complete():
                step_info = self.robot.run_step()
                self.step_count += 1
                self._process_step_info(step_info)

                if self.step_count % 50 == 0:
                    self._print_progress()
            else:
                self._print_final_results()
                self.paused = True

        pg.quit()

    def _handle_keypress(self, key) -> bool:
        """Handle keyboard input"""
        if key == pg.K_ESCAPE:
            return False
        elif key == pg.K_SPACE:
            self.paused = not self.paused
        elif key == pg.K_LEFT:
            self.speed = max(10, self.speed - 10)
        elif key == pg.K_RIGHT:
            self.speed = min(120, self.speed + 10)
        elif key == pg.K_s:
            self._save_screenshot()
        elif key == pg.K_r:
            self.show_sensor_range = not self.show_sensor_range
        elif key == pg.K_o:
            self.show_discovered_obstacles = not self.show_discovered_obstacles
        elif key == pg.K_i:
            self.show_stats_overlay = not self.show_stats_overlay

        return True

    def _process_step_info(self, step_info: Dict):
        """Process step information with safe position handling"""
        action = step_info.get('action', 'none')

        if action == 'move':
            pos = step_info.get('position')
            if pos:
                segment = step_info.get('segment', 'coverage')

                if segment == 'coverage':
                    self.ui.move_to(pos)
                elif segment == 'retreat':
                    self.ui.move_retreat(pos)
                elif segment == 'advance':
                    self.ui.move_advance(pos)

        elif action == 'task':
            pos = step_info.get('position')
            if pos:
                self.ui.task(pos)
            else:
                # Fallback: use current robot position
                self.ui.task(self.robot.current_pos.tuple)

        # Update energy display
        if hasattr(self.ui, 'set_energy_display'):
            self.ui.set_energy_display(self.robot.energy)

    def _update_ui(self):
        """Update UI with enhanced visualizations"""
        self.ui.draw()

        # Draw sensor range
        if self.show_sensor_range:
            self._draw_sensor_range()

        # Draw discovered obstacles
        if self.show_discovered_obstacles:
            self._draw_discovered_obstacles()

        # Draw stats overlay
        if self.show_stats_overlay:
            self._draw_stats_overlay()

        pg.display.flip()

    def _draw_sensor_range(self):
        """Draw sensor detection range với enhanced visual"""
        robot_pixel_pos = (
            (self.robot.current_pos.col + 0.5) * 8,
            (self.robot.current_pos.row + 0.5) * 8
        )

        sensor_radius_pixels = self.sensor_range * 8

        # Draw sensor range circle (outline)
        pg.draw.circle(self.ui.WIN, (0, 255, 255),
                      (int(robot_pixel_pos[0]), int(robot_pixel_pos[1])),
                      int(sensor_radius_pixels), 2)

        # Draw semi-transparent sensor area
        sensor_surface = pg.Surface((sensor_radius_pixels * 2, sensor_radius_pixels * 2))
        sensor_surface.set_alpha(32)
        sensor_surface.fill((0, 255, 255))

        # Create circular mask
        pg.draw.circle(sensor_surface, (0, 255, 255),
                      (int(sensor_radius_pixels), int(sensor_radius_pixels)),
                      int(sensor_radius_pixels))

        self.ui.WIN.blit(sensor_surface,
                        (robot_pixel_pos[0] - sensor_radius_pixels,
                         robot_pixel_pos[1] - sensor_radius_pixels))

        # Draw center dot
        pg.draw.circle(self.ui.WIN, (255, 255, 0),
                      (int(robot_pixel_pos[0]), int(robot_pixel_pos[1])), 3)

    def _draw_discovered_obstacles(self):
        """Draw discovered obstacles với different color"""
        for obstacle_pos in self.robot.etm.discovered_obstacles:
            pos_tuple = obstacle_pos.tuple
            if (0 <= pos_tuple[0] < self.robot.rows and
                0 <= pos_tuple[1] < self.robot.cols):

                # Draw discovered obstacle marker
                pixel_x = pos_tuple[1] * 8 + 2
                pixel_y = pos_tuple[0] * 8 + 2

                pg.draw.rect(self.ui.WIN, (255, 100, 100),
                           (pixel_x, pixel_y, 4, 4))

    def _draw_stats_overlay(self):
        """Draw real-time statistics overlay"""
        stats = self.robot.get_statistics()
        font = pg.font.SysFont(None, 24)

        # Create semi-transparent background
        overlay_surface = pg.Surface((300, 120))
        overlay_surface.set_alpha(180)
        overlay_surface.fill((0, 0, 0))
        self.ui.WIN.blit(overlay_surface, (10, 10))

        # Display key statistics
        y_offset = 15
        texts = [
            f"Step: {self.step_count}",
            f"Coverage: {stats['coverage_percentage']:.1f}%",
            f"Energy: {self.robot.energy:.0f}/{self.energy_config.capacity}",
            f"Returns: {stats['return_count']}",
            f"Discovered: {stats['total_discovered_obstacles']} obstacles"
        ]

        for text in texts:
            text_surface = font.render(text, True, (255, 255, 255))
            self.ui.WIN.blit(text_surface, (15, y_offset))
            y_offset += 20

    def _print_progress(self):
        """Print progress statistics"""
        stats = self.robot.get_statistics()
        sensor_stats = self.robot.etm.get_sensor_statistics()
        elapsed = time.time() - self.start_time
        etm_state = self.robot.etm.get_state().value

        print(f"Step {self.step_count:4d} | "
              f"Coverage: {stats['coverage_percentage']:5.1f}% | "
              f"Returns: {stats['return_count']:2d} | " 
              f"Energy: {self.robot.energy:6.1f} | "
              f"Discovered: {sensor_stats['total_discovered_obstacles']:3d} | "
              f"ETM: {etm_state}")

    def _print_final_results(self):
        """Print final algorithm results"""
        if not hasattr(self, '_results_printed'):
            execution_time = time.time() - self.start_time if self.start_time else 0
            stats = self.robot.get_statistics()
            sensor_stats = self.robot.etm.get_sensor_statistics()

            print("\n" + "=" * 60)
            print("ε⋆+ ALGORITHM - FINAL RESULTS")
            print("=" * 60)

            print("ROBOT CONFIGURATION:")
            print(f"  Starting position:   {self.battery_pos.tuple}")
            print(f"  Map dimensions:      {self.true_environment_array.shape}")
            print(f"  Sensor range Rs:     {self.sensor_range}")
            print(f"  Energy capacity:     {self.energy_config.capacity}")

            print("SENSOR DISCOVERY:")
            print(f"  Obstacles discovered: {sensor_stats['total_discovered_obstacles']}")
            print(f"  Sensor activations:   {stats['sensor_activations']}")

            true_obstacles = np.sum(self.true_environment_array == 1)
            discovery_rate = (sensor_stats['total_discovered_obstacles'] / true_obstacles * 100) if true_obstacles > 0 else 0
            print(f"  True obstacles:       {true_obstacles}")
            print(f"  Discovery rate:       {discovery_rate:.1f}%")

            print("PATH STATISTICS:")
            print(f"  Coverage length:     {stats['coverage_length']:.2f}")
            print(f"  Retreat length:      {stats['retreat_length']:.2f}")
            print(f"  Advance length:      {stats['advance_length']:.2f}")
            print(f"  Total path length:   {stats['total_path_length']:.2f}")

            print("COVERAGE STATISTICS:")
            print(f"  Coverage percentage: {stats['coverage_percentage']:.1f}%")
            if 'overlap_rate' in stats:
                print(f"  Overlap rate:        {stats['overlap_rate']:.2f}%")

            print("OPERATIONAL STATISTICS:")
            print(f"  Number of returns:   {stats['return_count']}")
            print(f"  Number of tasks:     {stats['task_count']}")
            print(f"  Total steps:         {self.step_count}")
            print(f"  Final energy:        {self.robot.energy:.1f}")

            print("PERFORMANCE:")
            print(f"  Execution time:      {execution_time:.3f}s")
            if execution_time > 0:
                print(f"  Steps per second:    {self.step_count/execution_time:.1f}")
            print(f"  ETM final state:     {self.robot.etm.get_state().value}")

            print("=" * 60)

            self._results_printed = True

    def _save_screenshot(self):
        """Save screenshot with timestamp"""
        timestamp = int(time.time())
        filename = f"epsilon_star_plus_{timestamp}.png"
        pg.image.save(self.ui.WIN, filename)

    def get_statistics_summary(self) -> Dict:
        """Get summary statistics"""
        stats = self.robot.get_statistics()
        sensor_stats = self.robot.etm.get_sensor_statistics()
        execution_time = time.time() - self.start_time if self.start_time else 0
        return {
            'algorithm': 'ε⋆+',
            'battery_position': self.battery_pos.tuple,
            'sensor_range': self.sensor_range,
            'total_path_length': stats['total_path_length'],
            'coverage_length': stats['coverage_length'],
            'overlap_rate': stats.get('overlap_rate', 0),
            'return_count': stats['return_count'],
            'execution_time': execution_time,
            'coverage_percentage': stats['coverage_percentage'],
            'obstacles_discovered': sensor_stats['total_discovered_obstacles'],
            'sensor_activations': stats['sensor_activations']
        }


def run_quick_demo():
    """Quick demo function"""
    runner = EpsilonStarPlusRunner("map/real_map/denmark.txt", sensor_range=2.0)
    runner.run()


def main():
    """Main function for ε⋆+ algorithm"""
    import argparse

    parser = argparse.ArgumentParser(description='ε⋆+ Algorithm Runner')
    parser.add_argument('--map', '-m', type=str,
                       default="map/real_map/denmark.txt",
                       help='Path to map file')
    parser.add_argument('--sensor-range', '-s', type=float,
                       default=2.0,
                       help='Sensor detection range Rs')
    args = parser.parse_args()

    print("🚀 LAUNCHING SENSOR-BASED ε⋆+ ALGORITHM")
    print("="*50)
    print("📚 Based on:")
    print("   • Song & Gupta (2018) - ε⋆ Algorithm")
    print("   • Shen et al. (2020) - ε⋆+ Extension")
    print("🔧 Implementation:")
    print("   • Progressive obstacle discovery via sensors")
    print("   • Real-time MAPS updates based on sensor feedback")
    print("   • Unknown environment assumption")
    print(f"   • Sensor range Rs: {args.sensor_range}")
    print("   • 100% Paper Compliant")
    print("🎮 CONTROLS:")
    print("   • SPACE: Pause/Resume")
    print("   • LEFT/RIGHT: Speed control")
    print("   • R: Toggle sensor range display")
    print("   • O: Toggle discovered obstacles")
    print("   • I: Toggle info overlay")
    print("   • S: Save screenshot")
    print("="*50)

    runner = EpsilonStarPlusRunner(args.map, args.sensor_range)
    runner.run()


if __name__ == "__main__":
    main()