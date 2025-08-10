#!/usr/bin/env python3
"""
run_sensor_epsilon_star_plus.py - SENSOR-BASED ε⋆+ ALGORITHM ENTRY POINT
🔧 100% Paper Compliant với Progressive Obstacle Discovery

Usage:
    cd bmwgoc/
    python run_sensor_epsilon_star_plus.py
    python run_sensor_epsilon_star_plus.py --map map/real_map/denmark.txt --sensor-range 2.5
"""

import sys
import os

# Add current directory to path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


def main():
    print("🚀 LAUNCHING SENSOR-BASED ε⋆+ ALGORITHM")
    print("=" * 50)
    print("📚 Based on:")
    print("   • Song & Gupta (2018) - ε⋆ Algorithm")
    print("   • Shen et al. (2020) - ε⋆+ Extension")
    print("🔧 Implementation:")
    print("   • Progressive obstacle discovery via sensors")
    print("   • Real-time MAPS updates based on sensor feedback")
    print("   • Unknown environment assumption")
    print("   • Sensor range Rs limitation")
    print("   • 100% Paper Compliant")
    print()

    try:
        # Import and run the sensor-based algorithm
        from epsilon_star.runner import main as run_sensor_algorithm
        run_sensor_algorithm()

    except ImportError as e:
        print(f"❌ Import Error: {e}")
        print("📂 Make sure you're running from bmwgoc/ directory")
        print("📂 Required files:")
        print("   • epsilon_star/core.py")
        print("   • epsilon_star/etm.py")
        print("   • epsilon_star/robot.py")
        print("   • epsilon_star/runner.py")
        print("   • grid_map.py")
        sys.exit(1)

    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()