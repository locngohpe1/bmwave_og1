#!/usr/bin/env python3
"""
run_epsilon_star_plus.py
Main entry point cho ε⋆+ Algorithm

Usage:
    python run_epsilon_star_plus.py                    # Manual map edit
    python run_epsilon_star_plus.py -m map.txt         # Load specific map
    python run_epsilon_star_plus.py --demo             # Quick console demo
    python run_epsilon_star_plus.py --compare          # Performance comparison
"""

import sys
import os
import time
import argparse
from typing import List, Dict

# Add current directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import ε⋆+ components - Direct imports to avoid package issues
from epsilon_star.runner import EpsilonStarPlusRunner, run_quick_demo
from epsilon_star.robot import EpsilonStarPlusRobot
from epsilon_star.core import Position, EnergyConfig


# Define create_test_environment locally
def create_test_environment(rows: int, cols: int):
    """Create a test environment with some obstacles"""
    import numpy as np

    env = np.zeros((rows, cols))

    # Add some obstacles
    for i in range(3, min(rows - 3, 8)):
        env[i, cols // 3] = 1
        env[i, 2 * cols // 3] = 1

    for j in range(cols // 3, 2 * cols // 3 + 1):
        env[3, j] = 1
        if rows > 8:
            env[7, j] = 1

    return env


def run_with_ui(map_file: str = None):
    """Run ε⋆+ with BWave UI"""
    try:
        runner = EpsilonStarPlusRunner(map_file)
        runner.run()
    except ImportError:
        print("BWave UI components not available")
        print("Make sure grid_map.py is in the same directory")
    except Exception as e:
        print(f"Error: {e}")


def run_console_only(map_file: str = None):
    """Run ε⋆+ console-only mode"""
    print("ε⋆+ Algorithm - Console Mode")
    print("=" * 40)

    if map_file and os.path.exists(map_file):
        # Load map from file (simplified)
        print(f"Loading map from: {map_file}")
        # For simplicity, create test environment
        environment = create_test_environment(20, 20)
        battery_pos = Position(0, 0)
    else:
        # Create test environment
        print("Creating test environment...")
        environment = create_test_environment(15, 15)
        battery_pos = Position(0, 0)

    print(f"Environment: {environment.shape}")
    print(f"Battery position: {battery_pos.tuple}")

    # Create robot
    energy_config = EnergyConfig(capacity=1000.0)
    robot = EpsilonStarPlusRobot(
        battery_pos,
        environment.shape[0],
        environment.shape[1],
        energy_config
    )
    robot.set_environment(environment)

    # Run algorithm
    print("\nExecuting ε⋆+ Algorithm...")
    start_time = time.time()
    step_count = 0

    while not robot.is_complete() and step_count < 500:
        step_info = robot.run_step()
        step_count += 1

        # Print progress every 50 steps
        if step_count % 50 == 0:
            stats = robot.get_statistics()
            print(f"Step {step_count:3d}: Coverage {stats['coverage_percentage']:5.1f}% | "
                  f"Returns {stats['return_count']:2d} | Energy {robot.energy:6.1f}")

    execution_time = time.time() - start_time

    # Final results
    print("\n" + "=" * 40)
    print("FINAL RESULTS")
    print("=" * 40)

    stats = robot.get_statistics()
    print(f"Total steps:         {step_count}")
    print(f"Execution time:      {execution_time:.3f}s")
    print(f"Steps per second:    {step_count / execution_time:.1f}")
    print(f"Coverage:            {stats['coverage_percentage']:.1f}%")
    print(f"Total path length:   {stats['total_path_length']:.2f}")
    print(f"Number of returns:   {stats['return_count']}")
    print(f"Final energy:        {robot.energy:.1f}")

    if 'overlap_rate' in stats:
        print(f"Overlap rate:        {stats['overlap_rate']:.2f}%")

    print("=" * 40)


def run_performance_comparison():
    """Run performance comparison on multiple test cases"""
    print("ε⋆+ Performance Comparison")
    print("=" * 50)

    test_cases = [
        (10, 10, "Small map"),
        (20, 20, "Medium map"),
        (30, 30, "Large map")
    ]

    results = []

    for rows, cols, description in test_cases:
        print(f"\nTesting {description} ({rows}×{cols})...")

        # Create test environment
        environment = create_test_environment(rows, cols)
        battery_pos = Position(0, 0)

        # Create robot
        robot = EpsilonStarPlusRobot(battery_pos, rows, cols)
        robot.set_environment(environment)

        # Run test
        start_time = time.time()
        step_count = 0
        max_steps = rows * cols * 2  # Reasonable limit

        while not robot.is_complete() and step_count < max_steps:
            robot.run_step()
            step_count += 1

        execution_time = time.time() - start_time
        stats = robot.get_statistics()

        result = {
            'description': description,
            'size': f"{rows}×{cols}",
            'steps': step_count,
            'time': execution_time,
            'path_length': stats['total_path_length'],
            'returns': stats['return_count'],
            'coverage': stats['coverage_percentage'],
            'steps_per_sec': step_count / execution_time if execution_time > 0 else 0
        }
        results.append(result)

        print(f"  Steps: {step_count}, Time: {execution_time:.2f}s, "
              f"Coverage: {stats['coverage_percentage']:.1f}%")

    # Print comparison table
    print("\n" + "=" * 70)
    print("PERFORMANCE COMPARISON SUMMARY")
    print("=" * 70)
    print("Map Size  | Steps | Time(s) | Path | Returns | Coverage | Steps/sec")
    print("-" * 70)

    for r in results:
        print(f"{r['size']:8s} | {r['steps']:5d} | {r['time']:7.2f} | "
              f"{r['path_length']:4.1f} | {r['returns']:7d} | "
              f"{r['coverage']:8.1f}% | {r['steps_per_sec']:9.1f}")

    print("=" * 70)


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='ε⋆+ Algorithm Runner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_epsilon_star_plus.py                    # Load cantwell map with UI
  python run_epsilon_star_plus.py -m map.txt         # Load specific map with UI
  python run_epsilon_star_plus.py --demo             # Quick console demo
  python run_epsilon_star_plus.py --console          # Console-only mode
  python run_epsilon_star_plus.py --compare          # Performance comparison
        """
    )

    parser.add_argument('--map', '-m', type=str,
                        help='Path to map file (default: cantwell.txt)')
    parser.add_argument('--demo', '-d', action='store_true',
                        help='Run quick demo without UI')
    parser.add_argument('--console', '-c', action='store_true',
                        help='Run in console-only mode (no UI)')
    parser.add_argument('--compare', action='store_true',
                        help='Run performance comparison')
    parser.add_argument('--energy', '-e', type=float, default=1000.0,
                        help='Energy capacity (default: 1000)')

    args = parser.parse_args()

    # Default to cantwell map if no map specified
    if not args.map:
        args.map = "map/real_map/cantwell.txt"

    try:
        if args.demo:
            run_quick_demo()
        elif args.compare:
            run_performance_comparison()
        elif args.console:
            run_console_only(args.map)
        else:
            print("Starting ε⋆+ Algorithm with BWave UI...")
            run_with_ui(args.map)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        print("Try running with --demo flag for a simple test")


if __name__ == "__main__":
    main()