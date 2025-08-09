#!/usr/bin/env python3
"""
run_epsilon_star_plus.py
Simplified entry point cho ε⋆+ Algorithm - CHỈ SỬ DỤNG REAL MAPS
"""

import sys
import os

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from epsilon_star.runner import EpsilonStarPlusRunner


def main():
    """
    Main entry point - CHỈ sử dụng Denmark map
    """

    # FIXED PATH - Denmark map only
    denmark_map_path = "map/real_map/denmark.txt"

    print("=" * 60)
    print("ε⋆+ ALGORITHM - COVERAGE PATH PLANNING")
    print("=" * 60)
    print(f"Map: {denmark_map_path}")
    print("Algorithm: ε⋆+ (Shen et al. 2020)")
    print("Environment: Energy-constrained coverage")
    print("=" * 60)

    try:
        # Create and run
        runner = EpsilonStarPlusRunner(denmark_map_path)
        runner.run()

    except FileNotFoundError as e:
        print(f"\n❌ ERROR: {e}")
        print(f"Please ensure the map file exists at: {denmark_map_path}")
        return 1

    except Exception as e:
        print(f"\n❌ UNEXPECTED ERROR: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)