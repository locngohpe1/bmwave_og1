"""
epsilon_star/runner.py - FIXED VERSION
BWave Integration Runner cho ε⋆+ Algorithm với UI method calls được fix
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
    Runner class để integrate ε⋆+ với BWave framework
    """

    def __init__(self, map_file: str = "map/real_map/denmark.txt"):
        # Initialize UI
        self.ui = Grid_Map()

        # Initialize pygame first
        import pygame as pg
        pg.init()

        # Load map - ONLY real maps, no fallbacks
        if not map_file:
            map_file = "map/real_map/denmark.txt"

        if not os.path.exists(map_file):
            raise FileNotFoundError(f"Map file not found: {map_file}")

        print(f"📂 Loading map from: {map_file}")
        environment, battery_pos_tuple = self.ui.read_map(map_file)

        # Ensure UI is properly updated with loaded map
        self.ui.map = np.array(environment, dtype=object)
        self.ui.row_count = len(environment)
        self.ui.col_count = len(environment[0])

        print(f"📏 Map dimensions: {self.ui.row_count} x {self.ui.col_count}")
        print(f"🔍 Map preview (first few values): {environment[0][:10] if len(environment) > 0 else 'Empty'}")

        # ✅ INTERACTIVE SETUP PHASE
        print(f"🎮 Starting interactive setup...")
        environment, battery_pos_tuple = self.ui.edit_map()

        self.environment_array = np.array(environment)
        self.battery_pos = Position(battery_pos_tuple[0], battery_pos_tuple[1])

        # Energy configuration theo paper ε⋆+
        self.energy_config = EnergyConfig(
            capacity=1000.0,
            coverage_rate=2.0,    # "twice this amount for coverage segment"
            advance_rate=1.0,     # "proportional to trajectory length"
            retreat_rate=1.0
        )

        # Create robot
        self.robot = EpsilonStarPlusRobot(
            self.battery_pos,
            len(self.environment_array),
            len(self.environment_array[0]),
            self.energy_config
        )

        # Set environment
        self.robot.set_environment(self.environment_array)

        # Control variables
        self.paused = False
        self.speed = 60  # FPS
        self.step_count = 0
        self.start_time = None

        print(f"✅ Map loaded: {self.environment_array.shape}")
        print(f"🔋 Battery position: {self.battery_pos.tuple}")
        print(f"⚡ Energy capacity: {self.energy_config.capacity}")
        print(f"📊 Coverage rate: {self.energy_config.coverage_rate}x")
        print(f"🏃 Advance/Retreat rate: {self.energy_config.advance_rate}x")
        print("\n🎮 CONTROLS:")
        print("SPACE: Pause/Resume | LEFT/RIGHT: Speed | S: Screenshot | ESC: Exit")
        print("=" * 60)

    def run(self):
        """Main execution loop"""
        clock = pg.time.Clock()
        running = True
        self.start_time = time.time()

        print(f"\n🚀 Starting ε⋆+ Algorithm on Denmark map...")
        print("🔄 ETM State Machine: ST → CP0 → WT → FN")
        print("📈 Real-time statistics will be shown every 100 steps\n")

        while running:
            # Handle events
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    running = False
                elif event.type == pg.KEYDOWN:
                    running = self._handle_keypress(event.key)

            # Update UI
            self._update_ui()
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
        print("Algorithm terminated.")

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

        return True

    def _process_step_info(self, step_info: Dict):
        """
        🔧 FIXED: Process information from algorithm step
        All UI method calls now pass tuples correctly
        """
        action = step_info.get('action', 'none')

        if action == 'move':
            pos = step_info['position']  # pos is already a tuple (row, col)
            segment = step_info['segment']

            if segment == 'coverage':
                # ✅ FIXED: Pass tuple directly instead of unpacking
                self.ui.move_to(pos)
            elif segment == 'retreat':
                # ✅ FIXED: Pass tuple directly instead of unpacking
                self.ui.move_retreat(pos)
            elif segment == 'advance':
                # ✅ FIXED: Pass tuple directly instead of unpacking
                self.ui.move_advance(pos)

        elif action == 'task':
            pos = step_info['position']  # pos is already a tuple (row, col)
            # ✅ FIXED: Pass tuple directly instead of unpacking
            self.ui.task(pos)

        elif action == 'energy_cycle':
            print(f"🔋 Energy cycle #{self.robot.stats['return_count']} completed - "
                  f"Retreat → Charge → Advance")

        # Update UI energy display
        if hasattr(self.ui, 'set_energy_display'):
            self.ui.set_energy_display(self.robot.energy)

    def _update_ui(self):
        """Update UI visualization"""
        self.ui.draw()

    def _print_progress(self):
        """Print progress information"""
        stats = self.robot.get_statistics()
        elapsed = time.time() - self.start_time
        etm_state = self.robot.etm.get_state().value

        print(f"📊 Step {self.step_count:4d} | "
              f"Coverage: {stats['coverage_percentage']:5.1f}% | "
              f"Returns: {stats['return_count']:2d} | " 
              f"Energy: {self.robot.energy:6.1f} | "
              f"ETM: {etm_state} | "
              f"Time: {elapsed:6.1f}s")

    def _print_final_results(self):
        """Print final results"""
        if not hasattr(self, '_results_printed'):
            execution_time = time.time() - self.start_time if self.start_time else 0
            stats = self.robot.get_statistics()

            print("\n" + "🏆" + "=" * 48 + "🏆")
            print("           ε⋆+ ALGORITHM - FINAL RESULTS")
            print("🏆" + "=" * 48 + "🏆")

            # Path statistics
            print("📏 PATH STATISTICS:")
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

            print("🏆" + "=" * 48 + "🏆")

            self._results_printed = True

    def _save_screenshot(self):
        """Save screenshot"""
        timestamp = int(time.time())
        filename = f"epsilon_star_denmark_{timestamp}.png"
        pg.image.save(self.ui.WIN, filename)
        print(f"📸 Screenshot saved: {filename}")

    def get_statistics_summary(self) -> Dict:
        """Get summary statistics"""
        stats = self.robot.get_statistics()
        execution_time = time.time() - self.start_time if self.start_time else 0

        return {
            'algorithm': 'ε⋆+',
            'total_path_length': stats['total_path_length'],
            'coverage_length': stats['coverage_length'],
            'overlap_rate': stats.get('overlap_rate', 0),
            'return_count': stats['return_count'],
            'execution_time': execution_time,
            'coverage_percentage': stats['coverage_percentage']
        }


def main():
    """Main function - Only real maps, no demos"""
    import argparse

    parser = argparse.ArgumentParser(description='ε⋆+ Algorithm Runner')
    parser.add_argument('--map', '-m', type=str,
                       default="map/real_map/denmark.txt",
                       help='Path to map file (default: map/real_map/denmark.txt)')

    args = parser.parse_args()

    runner = EpsilonStarPlusRunner(args.map)
    runner.run()


if __name__ == "__main__":
    main()