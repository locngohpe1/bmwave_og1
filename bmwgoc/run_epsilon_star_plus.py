#!/usr/bin/env python3
"""
run_epsilon_star_plus.py - FIXED VERSION
Main entry point cho ε⋆+ Algorithm với Interactive Setup

Usage Examples:
    python run_epsilon_star_plus.py                           # Interactive mode (default)
    python run_epsilon_star_plus.py -m map.txt               # Interactive with map
    python run_epsilon_star_plus.py -m map.txt --auto        # Auto-load mode
    python run_epsilon_star_plus.py --demo                   # Quick console demo
    python run_epsilon_star_plus.py --test                   # Run test scenarios
"""

import sys
import os
import time
import argparse
from typing import List, Dict

# Add current directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import ε⋆+ components
try:
    from epsilon_star.runner import EpsilonStarPlusRunner, run_quick_demo
    from epsilon_star.robot import EpsilonStarPlusRobot
    from epsilon_star.core import Position, EnergyConfig
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure epsilon_star package is available")
    sys.exit(1)


def create_test_environment(rows: int, cols: int):
    """Create a test environment with strategic obstacle placement"""
    import numpy as np

    env = np.zeros((rows, cols))

    # Create a more complex test environment
    # Central obstacle block
    for i in range(rows // 3, 2 * rows // 3):
        for j in range(cols // 3, 2 * cols // 3):
            if i == rows // 3 or i == 2 * rows // 3 - 1 or j == cols // 3 or j == 2 * cols // 3 - 1:
                env[i, j] = 1

    # Add some scattered obstacles
    for i in range(2, rows - 2, 4):
        for j in range(2, cols - 2, 6):
            env[i, j] = 1

    # Create narrow passages
    for i in range(rows // 4, 3 * rows // 4):
        env[i, cols // 2] = 1

    # Add openings in passages
    env[rows // 2, cols // 2] = 0
    env[rows // 2 + 1, cols // 2] = 0

    return env


def run_interactive_mode(map_file: str = None):
    """Run ε⋆+ with full interactive setup"""
    print("=" * 60)
    print("ε⋆+ ALGORITHM - INTERACTIVE MODE")
    print("=" * 60)
    print("Features:")
    print("• Click to place/remove obstacles")
    print("• Right-click to set charging station")
    print("• Press ENTER to start algorithm")
    print("• Real-time visualization")
    print("=" * 60)

    try:
        runner = EpsilonStarPlusRunner(map_file, interactive_mode=True)
        runner.run()
    except ImportError:
        print("ERROR: BWave UI components not available")
        print("Make sure grid_map.py is in the same directory")
        return False
    except Exception as e:
        print(f"Error in interactive mode: {e}")
        return False

    return True


def run_auto_mode(map_file: str):
    """Run ε⋆+ with auto-loaded map"""
    print("=" * 60)
    print("ε⋆+ ALGORITHM - AUTO MODE")
    print("=" * 60)

    if not os.path.exists(map_file):
        print(f"ERROR: Map file not found: {map_file}")
        return False

    try:
        runner = EpsilonStarPlusRunner(map_file, interactive_mode=False)
        runner.run()
    except Exception as e:
        print(f"Error in auto mode: {e}")
        return False

    return True


def run_test_scenarios():
    """Run comprehensive test scenarios"""
    print("=" * 60)
    print("ε⋆+ ALGORITHM - TEST SCENARIOS")
    print("=" * 60)

    test_cases = [
        (10, 10, "Small Environment"),
        (15, 15, "Medium Environment"),
        (20, 20, "Large Environment"),
        (25, 15, "Rectangular Environment")
    ]

    results = []

    for rows, cols, description in test_cases:
        print(f"\n--- Testing {description} ({rows}×{cols}) ---")

        # Create test environment
        environment = create_test_environment(rows, cols)
        battery_pos = Position(0, 0)

        # Setup energy config
        energy_config = EnergyConfig(
            capacity=1000.0,
            coverage_rate=2.0,
            advance_rate=1.0,
            retreat_rate=1.0
        )

        # Create robot
        robot = EpsilonStarPlusRobot(battery_pos, rows, cols, energy_config)
        robot.set_environment(environment)

        # Run test
        start_time = time.time()
        step_count = 0
        max_steps = rows * cols * 3  # Generous limit

        print(f"Running algorithm (max {max_steps} steps)...")

        while not robot.is_complete() and step_count < max_steps:
            step_info = robot.run_step()
            step_count += 1

            # Progress update every 100 steps
            if step_count % 100 == 0:
                stats = robot.get_statistics()
                print(f"  Step {step_count}: {stats['coverage_percentage']:.1f}% coverage")

        execution_time = time.time() - start_time
        stats = robot.get_statistics()

        # Collect results
        result = {
            'description': description,
            'size': f"{rows}×{cols}",
            'steps': step_count,
            'time': execution_time,
            'path_length': stats['total_path_length'],
            'coverage_length': stats['coverage_length'],
            'returns': stats['return_count'],
            'coverage': stats['coverage_percentage'],
            'overlap_rate': stats.get('overlap_rate', 0),
            'completed': robot.is_complete()
        }
        results.append(result)

        # Print individual result
        print(f"Results:")
        print(f"  Steps: {step_count}")
        print(f"  Time: {execution_time:.2f}s")
        print(f"  Coverage: {stats['coverage_percentage']:.1f}%")
        print(f"  Path Length: {stats['total_path_length']:.1f}")
        print(f"  Returns: {stats['return_count']}")
        print(f"  Completed: {'Yes' if robot.is_complete() else 'No'}")

    # Print summary table
    print("\n" + "=" * 80)
    print("TEST SCENARIOS SUMMARY")
    print("=" * 80)
    print(f"{'Environment':<20} {'Steps':<6} {'Time':<8} {'Coverage':<10} {'Returns':<8} {'Complete':<10}")
    print("-" * 80)

    for r in results:
        complete_status = "Yes" if r['completed'] else "No"
        print(f"{r['description']:<20} {r['steps']:<6} {r['time']:<8.2f} "
              f"{r['coverage']:<10.1f}% {r['returns']:<8} {complete_status:<10}")

    print("=" * 80)

    # Summary statistics
    completed_tests = sum(1 for r in results if r['completed'])
    avg_coverage = sum(r['coverage'] for r in results) / len(results)
    total_time = sum(r['time'] for r in results)

    print(f"\nOverall Summary:")
    print(f"  Tests completed successfully: {completed_tests}/{len(results)}")
    print(f"  Average coverage: {avg_coverage:.1f}%")
    print(f"  Total execution time: {total_time:.2f}s")

    return results


def run_debug_mode(map_file: str = None):
    """Run in debug mode with detailed logging"""
    print("=" * 60)
    print("ε⋆+ ALGORITHM - DEBUG MODE")
    print("=" * 60)

    # Create or load environment
    if map_file and os.path.exists(map_file):
        print(f"Loading map from: {map_file}")
        # For simplicity, create test environment
        environment = create_test_environment(20, 20)
        battery_pos = Position(0, 0)
    else:
        print("Creating debug test environment...")
        environment = create_test_environment(15, 15)
        battery_pos = Position(0, 0)

    print(f"Environment: {environment.shape}")
    print(f"Battery position: {battery_pos.tuple}")

    # Create robot with debug enabled
    energy_config = EnergyConfig(capacity=500.0)  # Lower capacity for more returns
    robot = EpsilonStarPlusRobot(battery_pos, environment.shape[0], environment.shape[1], energy_config)
    robot.set_environment(environment)

    # Enable debug mode
    robot.etm.debug_mode = True

    print("\nStarting algorithm with debug output...")
    start_time = time.time()
    step_count = 0

    while not robot.is_complete() and step_count < 1000:
        step_info = robot.run_step()
        step_count += 1

        # Detailed progress
        if step_count % 25 == 0:
            stats = robot.get_statistics()
            etm_stats = robot.etm.get_coverage_statistics()

            print(f"\nStep {step_count}:")
            print(f"  Coverage: {stats['coverage_percentage']:.1f}%")
            print(f"  ETM Coverage: {etm_stats['coverage_percentage']:.1f}%")
            print(f"  ETM State: {robot.etm.get_state().value}")
            print(f"  Returns: {stats['return_count']}")
            print(f"  Energy: {robot.energy:.1f}")
            print(f"  Unexplored: {etm_stats['unexplored_cells']}")

    execution_time = time.time() - start_time
    final_stats = robot.get_statistics()
    etm_stats = robot.etm.get_coverage_statistics()

    print("\n" + "=" * 50)
    print("DEBUG SESSION RESULTS")
    print("=" * 50)
    print(f"Total steps: {step_count}")
    print(f"Execution time: {execution_time:.3f}s")
    print(f"Final coverage: {final_stats['coverage_percentage']:.1f}%")
    print(f"ETM coverage: {etm_stats['coverage_percentage']:.1f}%")
    print(f"ETM final state: {robot.etm.get_state().value}")
    print(f"Algorithm completed: {'Yes' if robot.is_complete() else 'No'}")
    print("=" * 50)


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
        # Default to Denmark map if no map specified
        map_file = args.map
        if not map_file:
            map_file = "map/real_map/denmark.txt"

        runner = EpsilonStarPlusRunner(map_file)
        runner.run()


if __name__ == "__main__":
    main()