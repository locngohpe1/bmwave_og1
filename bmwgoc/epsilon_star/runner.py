"""
epsilon_star/runner.py
BWave Integration Runner
Chạy ε⋆+ Algorithm trong môi trường BWave
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

    def __init__(self, map_file: str = None):
        # Initialize UI
        self.ui = Grid_Map()

        # Load map
        if map_file and os.path.exists(map_file):
            environment, battery_pos_tuple = self.ui.read_map(map_file)
        else:
            print("Map file not found, using manual edit mode...")
            # Initialize pygame first for edit mode
            import pygame as pg
            pg.init()
            # Set default map size for edit mode
            self.ui.row_count = 20
            self.ui.col_count = 20
            self.ui.map = [[0 for _ in range(20)] for _ in range(20)]
            environment, battery_pos_tuple = self.ui.edit_map()

        self.environment_array = np.array(environment)
        self.battery_pos = Position(battery_pos_tuple[0], battery_pos_tuple[1])

        # Energy configuration
        self.energy_config = EnergyConfig(
            capacity=1000.0,
            coverage_rate=2.0,
            advance_rate=1.0,
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

        print(f"Map loaded: {self.environment_array.shape}")
        print(f"Battery position: {self.battery_pos.tuple}")
        print("\nControls:")
        print("SPACE: Pause/Resume | LEFT/RIGHT: Speed | S: Screenshot | ESC: Exit")

    def run(self):
        """Main execution loop"""
        clock = pg.time.Clock()
        running = True
        self.start_time = time.time()

        print("\nStarting ε⋆+ Algorithm...")

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
        """Process information from algorithm step"""
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
            print(f"Energy cycle #{self.robot.stats['return_count']} completed")

        # Update UI energy display
        self.ui.set_energy_display(self.robot.energy)

    def _update_ui(self):
        """Update UI visualization"""
        self.ui.draw()

    def _print_progress(self):
        """Print progress information"""
        stats = self.robot.get_statistics()
        elapsed = time.time() - self.start_time

        print(f"Step {self.step_count:4d} | Coverage: {stats['coverage_percentage']:5.1f}% | "
              f"Returns: {stats['return_count']:2d} | Energy: {self.robot.energy:6.1f} | "
              f"Time: {elapsed:6.1f}s")

    def _print_final_results(self):
        """Print final results"""
        execution_time = time.time() - self.start_time
        stats = self.robot.get_statistics()

        print("\n" + "=" * 50)
        print("ε⋆+ ALGORITHM - FINAL RESULTS")
        print("=" * 50)

        # Path statistics
        print("PATH STATISTICS:")
        print(f"  Coverage length:     {stats['coverage_length']:8.2f}")
        print(f"  Retreat length:      {stats['retreat_length']:8.2f}")
        print(f"  Advance length:      {stats['advance_length']:8.2f}")
        print(f"  Total path length:   {stats['total_path_length']:8.2f}")

        # Coverage statistics
        print("\nCOVERAGE STATISTICS:")
        print(f"  Coverage percentage: {stats['coverage_percentage']:8.1f}%")
        if 'overlap_rate' in stats:
            print(f"  Overlap rate:        {stats['overlap_rate']:8.2f}%")

        # Operational statistics
        print("\nOPERATIONAL STATISTICS:")
        print(f"  Number of returns:   {stats['return_count']:8d}")
        print(f"  Number of tasks:     {stats['task_count']:8d}")
        print(f"  Total steps:         {self.step_count:8d}")
        print(f"  Final energy:        {self.robot.energy:8.1f}")

        # Performance statistics
        print("\nPERFORMANCE STATISTICS:")
        print(f"  Execution time:      {execution_time:8.3f}s")
        print(f"  Steps per second:    {self.step_count/execution_time:8.1f}")
        print(f"  ETM final state:     {self.robot.etm.get_state().value}")

        print("=" * 50)

    def _save_screenshot(self):
        """Save screenshot"""
        timestamp = int(time.time())
        filename = f"epsilon_star_plus_{timestamp}.png"
        pg.image.save(self.ui.WIN, filename)
        print(f"Screenshot saved: {filename}")

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


def run_quick_demo():
    """Quick demo without UI"""
    from .robot import EpsilonStarPlusRobot
    from .core import Position, EnergyConfig

    print("ε⋆+ Algorithm - Quick Demo (Console Only)")
    print("=" * 40)

    # Simple 10x10 environment
    environment = np.zeros((10, 10))
    environment[3:7, 3:7] = 1  # Central obstacle block

    battery_pos = Position(0, 0)
    robot = EpsilonStarPlusRobot(battery_pos, 10, 10)
    robot.set_environment(environment)

    print("Running on 10×10 environment with central obstacle...")

    start_time = time.time()
    steps = 0
    while not robot.is_complete() and steps < 200:
        robot.run_step()
        steps += 1

    execution_time = time.time() - start_time
    stats = robot.get_statistics()

    print(f"Completed: {stats['coverage_percentage']:.1f}% coverage in {steps} steps")
    print(f"Path length: {stats['total_path_length']:.1f}")
    print(f"Returns: {stats['return_count']}")
    print(f"Execution time: {execution_time:.3f}s")


def main():
    """Main function"""
    import argparse

    parser = argparse.ArgumentParser(description='ε⋆+ Algorithm Runner')
    parser.add_argument('--map', '-m', type=str,
                       help='Path to map file')
    parser.add_argument('--demo', '-d', action='store_true',
                       help='Run quick demo without UI')

    args = parser.parse_args()

    if args.demo:
        run_quick_demo()
    else:
        runner = EpsilonStarPlusRunner(args.map)
        runner.run()


if __name__ == "__main__":
    main()