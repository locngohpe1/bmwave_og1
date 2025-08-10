"""
runner.py - SENSOR-BASED RUNNER IMPLEMENTATION
🔧 100% Paper Compliant với Progressive Obstacle Discovery
"""

import numpy as np
import pygame as pg
import time
import sys
import os
from typing import Dict

# Import BWave components
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from grid_map import Grid_Map

# Import ε⋆+ components
from .core import Position, EnergyConfig
from .robot import EpsilonStarPlusRobot


class EpsilonStarPlusRunner:
    """
    🔧 SENSOR-BASED Runner - 100% Paper Compliant
    - Progressive obstacle discovery within sensor range
    - Unknown environment assumption
    - Real-time MAPS updates based on sensor feedback
    """

    def __init__(self, map_file: str = "map/real_map/denmark.txt", sensor_range: float = 2.0):
        # Initialize UI
        self.ui = Grid_Map()

        # Initialize pygame
        import pygame as pg
        pg.init()

        if not os.path.exists(map_file):
            raise FileNotFoundError(f"Map file not found: {map_file}")

        print(f"📂 Loading TRUE environment from: {map_file}")
        print(f"🔧 SENSOR-BASED MODE - Robot starts with UNKNOWN environment")

        # Load TRUE environment (hidden from robot)
        true_environment, _ = self.ui.read_map(map_file)

        # Update UI with true environment for visualization
        self.ui.map = np.array(true_environment, dtype=object)
        self.ui.row_count = len(true_environment)
        self.ui.col_count = len(true_environment[0])

        print(f"📏 Map dimensions: {self.ui.row_count} x {self.ui.col_count}")

        # Interactive setup for charging station
        print(f"🎮 Starting interactive setup...")
        print("⚡ IMPORTANT: Set charging station with RIGHT CLICK!")
        print("🌍 NOTE: Robot will discover obstacles progressively via sensors")

        final_environment, final_battery_pos = self.ui.edit_map()

        self.true_environment_array = np.array(final_environment)
        self.battery_pos = Position(final_battery_pos[0], final_battery_pos[1])

        print(f"✅ Interactive setup complete!")
        print(f"🔋 Charging station: {self.battery_pos.tuple}")

        # 🔧 SENSOR CONFIGURATION per papers
        self.sensor_range = sensor_range

        # Energy configuration theo paper ε⋆+
        self.energy_config = EnergyConfig(
            capacity=1000.0,      # Keep original capacity for paper compliance
            coverage_rate=2.0,    # "twice this amount for coverage segment"
            advance_rate=1.0,     # "proportional to trajectory length"
            retreat_rate=1.0
        )

        # Create sensor-based robot
        self.robot = EpsilonStarPlusRobot(
            self.battery_pos,
            len(self.true_environment_array),
            len(self.true_environment_array[0]),
            self.energy_config,
            sensor_range=self.sensor_range
        )

        # 🔧 Set TRUE environment for sensor simulation (robot doesn't know this)
        self.robot.set_environment(self.true_environment_array)

        # Control variables
        self.paused = False
        self.speed = 60
        self.step_count = 0
        self.start_time = None

        # Visualization state
        self.show_sensor_range = True
        self.show_discovered_obstacles = True

        print(f"📊 SENSOR-BASED Setup Summary:")
        print(f"   Map size: {self.true_environment_array.shape}")
        print(f"   Battery: {self.battery_pos.tuple}")
        print(f"   Sensor range Rs: {self.sensor_range}")
        print(f"   Energy capacity: {self.energy_config.capacity}")
        print(f"   🌍 Robot knowledge: UNKNOWN environment (will discover via sensors)")
        print("\n🎮 CONTROLS:")
        print("SPACE: Pause/Resume | LEFT/RIGHT: Speed | S: Screenshot | T: Toggle sensor viz | ESC: Exit")
        print("=" * 70)

    def run(self):
        """Main execution loop with sensor visualization"""
        clock = pg.time.Clock()
        running = True
        self.start_time = time.time()

        print(f"\n🚀 Starting SENSOR-BASED ε⋆+ Algorithm...")
        print(f"🔋 Robot starting from charging station: {self.battery_pos.tuple}")
        print(f"📡 Sensor range Rs = {self.sensor_range} (progressive obstacle discovery)")
        print(f"🌍 Unknown environment - robot will discover obstacles in real-time")
        print("📈 Real-time statistics will be shown every 100 steps\n")

        while running:
            # Handle events
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    running = False
                elif event.type == pg.KEYDOWN:
                    running = self._handle_keypress(event.key)

            # Update UI with sensor visualization
            self._update_ui_with_sensors()
            clock.tick(self.speed)

            if self.paused:
                continue

            # Execute algorithm step
            if not self.robot.is_complete():
                step_info = self.robot.run_step()
                self.step_count += 1
                self._process_step_info(step_info)

                # Progress updates
                if self.step_count % 100 == 0:
                    self._print_progress()
            else:
                # Coverage complete
                self._print_final_results()
                self.paused = True

        pg.quit()
        print("Sensor-based algorithm terminated.")

    def _handle_keypress(self, key) -> bool:
        """Handle keyboard input"""
        if key == pg.K_ESCAPE:
            return False
        elif key == pg.K_SPACE:
            self.paused = not self.paused
            status = "Paused" if self.paused else "Resumed"
            print(f"{status} - Step: {self.step_count}")
        elif key == pg.K_LEFT:
            self.speed = max(10, self.speed - 10)
            print(f"Speed: {self.speed} FPS")
        elif key == pg.K_RIGHT:
            self.speed = min(120, self.speed + 10)
            print(f"Speed: {self.speed} FPS")
        elif key == pg.K_s:
            self._save_screenshot()
        elif key == pg.K_t:
            self.show_sensor_range = not self.show_sensor_range
            print(f"Sensor visualization: {'ON' if self.show_sensor_range else 'OFF'}")

        return True

    def _process_step_info(self, step_info: Dict):
        """Process step information with sensor awareness"""
        action = step_info.get('action', 'none')

        if action == 'move':
            pos = step_info['position']
            segment = step_info['segment']

            if segment == 'coverage':
                self.ui.move_to(pos)
            elif segment == 'retreat':
                self.ui.move_retreat(pos)
            elif segment == 'advance':
                self.ui.move_advance(pos)

        elif action == 'task':
            pos = step_info['position']
            self.ui.task(pos)

        elif action == 'energy_cycle':
            print(f"🔋 Energy cycle #{self.robot.stats['return_count']} completed - "
                  f"Retreat → Charge → Advance")

        # Update UI energy display
        if hasattr(self.ui, 'set_energy_display'):
            self.ui.set_energy_display(self.robot.energy)

    def _update_ui_with_sensors(self):
        """Update UI with sensor range visualization"""
        self.ui.draw()

        # Draw sensor range if enabled
        if self.show_sensor_range:
            self._draw_sensor_range()

        # Draw discovered obstacles differently
        if self.show_discovered_obstacles:
            self._draw_discovered_obstacles()

    def _draw_sensor_range(self):
        """Draw sensor detection range around robot"""
        robot_pixel_pos = (
            (self.robot.current_pos.col + 0.5) * 8,  # EPSILON from grid_map
            (self.robot.current_pos.row + 0.5) * 8
        )

        sensor_radius_pixels = self.sensor_range * 8

        # Draw sensor range circle (semi-transparent)
        sensor_surface = pg.Surface((sensor_radius_pixels * 2, sensor_radius_pixels * 2))
        sensor_surface.set_alpha(64)  # Semi-transparent
        sensor_surface.fill((0, 255, 255))  # Cyan color

        pg.draw.circle(sensor_surface, (0, 255, 255),
                      (int(sensor_radius_pixels), int(sensor_radius_pixels)),
                      int(sensor_radius_pixels), 2)

        self.ui.WIN.blit(sensor_surface,
                        (robot_pixel_pos[0] - sensor_radius_pixels,
                         robot_pixel_pos[1] - sensor_radius_pixels))

    def _draw_discovered_obstacles(self):
        """Highlight discovered vs undiscovered obstacles"""
        # This would require access to robot's discovered_obstacles
        # For now, just indicate discovered area
        pass

    def _print_progress(self):
        """Print progress with sensor statistics"""
        stats = self.robot.get_statistics()
        sensor_stats = self.robot.etm.get_sensor_statistics()
        elapsed = time.time() - self.start_time
        etm_state = self.robot.etm.get_state().value

        print(f"📊 Step {self.step_count:4d} | "
              f"Coverage: {stats['coverage_percentage']:5.1f}% | "
              f"Returns: {stats['return_count']:2d} | " 
              f"Energy: {self.robot.energy:6.1f} | "
              f"Discovered: {sensor_stats['total_discovered_obstacles']:3d} obs | "
              f"ETM: {etm_state} | "
              f"Time: {elapsed:6.1f}s")

    def _print_final_results(self):
        """Print final results with sensor statistics"""
        if not hasattr(self, '_results_printed'):
            execution_time = time.time() - self.start_time if self.start_time else 0
            stats = self.robot.get_statistics()
            sensor_stats = self.robot.etm.get_sensor_statistics()

            print("\n" + "🏆" + "=" * 60 + "🏆")
            print("        SENSOR-BASED ε⋆+ ALGORITHM - FINAL RESULTS")
            print("🏆" + "=" * 60 + "🏆")

            # Robot configuration
            print("🤖 ROBOT CONFIGURATION:")
            print(f"   Starting position:   {self.battery_pos.tuple}")
            print(f"   Map dimensions:      {self.true_environment_array.shape}")
            print(f"   Sensor range Rs:     {self.sensor_range}")
            print(f"   Energy capacity:     {self.energy_config.capacity:8.1f}")

            # Sensor statistics
            print("\n📡 SENSOR DISCOVERY STATISTICS:")
            print(f"   Total obstacles discovered: {sensor_stats['total_discovered_obstacles']:8d}")
            print(f"   Sensor activations:         {stats['sensor_activations']:8d}")
            print(f"   Discovery events:           {sensor_stats['discovery_events']:8d}")

            # True vs discovered comparison
            true_obstacles = np.sum(self.true_environment_array == 1)
            discovery_rate = (sensor_stats['total_discovered_obstacles'] / true_obstacles * 100) if true_obstacles > 0 else 0
            print(f"   True obstacles in map:      {true_obstacles:8d}")
            print(f"   Discovery rate:             {discovery_rate:8.1f}%")

            # Path statistics
            print("\n📏 PATH STATISTICS:")
            print(f"   Coverage length:     {stats['coverage_length']:8.2f}")
            print(f"   Retreat length:      {stats['retreat_length']:8.2f}")
            print(f"   Advance length:      {stats['advance_length']:8.2f}")
            print(f"   Total path length:   {stats['total_path_length']:8.2f}")

            # Coverage statistics
            print("\n📊 COVERAGE STATISTICS:")
            print(f"   Coverage percentage: {stats['coverage_percentage']:8.1f}%")
            if 'overlap_rate' in stats:
                print(f"   Overlap rate:        {stats['overlap_rate']:8.2f}%")

            # Operational statistics
            print("\n⚙️  OPERATIONAL STATISTICS:")
            print(f"   Number of returns:   {stats['return_count']:8d}")
            print(f"   Number of tasks:     {stats['task_count']:8d}")
            print(f"   Total steps:         {self.step_count:8d}")
            print(f"   Final energy:        {self.robot.energy:8.1f}")

            # Performance statistics
            print("\n⏱️  PERFORMANCE STATISTICS:")
            print(f"   Execution time:      {execution_time:8.3f}s")
            if execution_time > 0:
                print(f"   Steps per second:    {self.step_count/execution_time:8.1f}")
            print(f"   ETM final state:     {self.robot.etm.get_state().value}")

            print("🏆" + "=" * 60 + "🏆")
            print("📡 SENSOR-BASED OPERATION COMPLETE")
            print("🌍 Robot successfully navigated unknown environment via progressive discovery")

            self._results_printed = True

    def _save_screenshot(self):
        """Save screenshot with sensor info"""
        timestamp = int(time.time())
        filename = f"sensor_epsilon_star_plus_{timestamp}.png"
        pg.image.save(self.ui.WIN, filename)
        print(f"📸 Screenshot saved: {filename}")

    def get_statistics_summary(self) -> Dict:
        """Get summary statistics including sensor data"""
        stats = self.robot.get_statistics()
        sensor_stats = self.robot.etm.get_sensor_statistics()
        execution_time = time.time() - self.start_time if self.start_time else 0

        return {
            'algorithm': 'Sensor-based ε⋆+',
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


def main():
    """Main function for sensor-based ε⋆+ algorithm"""
    import argparse

    parser = argparse.ArgumentParser(description='Sensor-based ε⋆+ Algorithm Runner')
    parser.add_argument('--map', '-m', type=str,
                       default="map/real_map/denmark.txt",
                       help='Path to map file (default: map/experiment/scenario1/map_1.txt)')
    parser.add_argument('--sensor-range', '-s', type=float,
                       default=2.0,
                       help='Sensor detection range Rs (default: 2.0)')

    args = parser.parse_args()

    print("🚀 SENSOR-BASED ε⋆+ Coverage Path Planning")
    print("=" * 50)
    print("📡 Progressive obstacle discovery via sensors")
    print("🌍 Unknown environment assumption")
    print("✅ 100% Paper Compliant Implementation")
    print()

    runner = EpsilonStarPlusRunner(args.map, args.sensor_range)
    runner.run()


if __name__ == "__main__":
    main()