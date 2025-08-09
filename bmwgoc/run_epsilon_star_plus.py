#!/usr/bin/env python3
"""
run_epsilon_star_plus.py - FIXED VERSION
Entry point cho ε⋆+ Algorithm với CORRECT import paths

✅ FIXES:
- Fixed import paths cho current directory structure
- Battery position now correctly set by user interaction
- No more auto-start from (0,0)
- Proper interactive setup flow
"""

import sys
import os

# Add current directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# ✅ FIXED: Import with correct path since we're running from bmwgoc/
try:
    from epsilon_star.runner import EpsilonStarPlusRunner
except ImportError as e:
    print(f"❌ Import Error: {e}")
    print("📁 Current working directory:", os.getcwd())
    print("🔍 Available files:")
    try:
        for item in os.listdir('.'):
            if item.endswith('.py'):
                print(f"   📄 {item}")
            elif os.path.isdir(item) and not item.startswith('.'):
                print(f"   📂 {item}/")
    except:
        pass

    print("\n🔧 Trying alternative import...")
    try:
        # Alternative import if running from different location
        sys.path.append('.')
        from epsilon_star.runner import EpsilonStarPlusRunner

        print("✅ Alternative import successful!")
    except ImportError as e2:
        print(f"❌ Alternative import also failed: {e2}")
        print("\n💡 SOLUTIONS:")
        print("1. Make sure you're running from bmwgoc/ directory")
        print("2. Check that epsilon_star/ directory exists")
        print("3. Verify all __init__.py files are present")
        sys.exit(1)


def main():
    """
    Main entry point för ε⋆+ Algorithm

    🔧 FIXED INITIALIZATION FLOW:
    1. Load Denmark map structure
    2. Interactive setup (RIGHT CLICK to set charging station!)
    3. Robot starts from USER-SELECTED charging station
    4. Complete coverage với energy constraints
    """
    print("🚀 ε⋆+ Coverage Path Planning Algorithm")
    print("=" * 50)
    print("📋 Algorithm: Shen et al. (2020) - Energy-Constrained")
    print("🗺️  Map: Denmark indoor environment")
    print("🔧 Status: FIXED battery position initialization")
    print("📁 Running from:", os.getcwd())
    print()

    print("🎮 SETUP INSTRUCTIONS:")
    print("1. ✅ Map will load automatically")
    print("2. 🖱️  LEFT CLICK: Add/remove obstacles")
    print("3. 🖱️  RIGHT CLICK: Set charging station (REQUIRED!)")
    print("4. ⌨️  PRESS ANY KEY: Start algorithm")
    print()
    print("⚠️  IMPORTANT: You MUST set charging station with RIGHT CLICK!")
    print("   Otherwise robot will start from default (0,0)")
    print()

    # Check if Denmark map exists
    map_path = "map/experiment/scenario1/map_1.txt"
    if not os.path.exists(map_path):
        print(f"❌ Map not found: {map_path}")
        print("📁 Current directory:", os.getcwd())

        # Check for alternative paths
        possible_paths = [
            "map/real_map/denmark.txt",
            "./map/real_map/denmark.txt",
            "../map/real_map/denmark.txt",
            "bmwgoc/map/real_map/denmark.txt"
        ]

        found = False
        for path in possible_paths:
            if os.path.exists(path):
                map_path = path
                print(f"✅ Found Denmark map at: {path}")
                found = True
                break

        if not found:
            print("❌ Cannot find Denmark map in any expected location")
            print("🔍 Checking current directory structure...")
            if os.path.exists("map"):
                print("   📂 map/ directory found")
                if os.path.exists("map/real_map"):
                    print("   📂 map/real_map/ directory found")
                    files = os.listdir("map/real_map")
                    print(f"   📋 Files in map/real_map/: {files}")
                else:
                    print("   ❌ map/real_map/ directory not found")
            else:
                print("   ❌ map/ directory not found")
            return 1

    try:
        print(f"🗺️  Loading map from: {map_path}")

        # Run with fixed initialization
        runner = EpsilonStarPlusRunner(map_path)
        runner.run()

        # Print summary
        print("\n📊 EXECUTION SUMMARY:")
        summary = runner.get_statistics_summary()
        for key, value in summary.items():
            print(f"   {key}: {value}")

        return 0

    except FileNotFoundError as e:
        print(f"❌ File Error: {e}")
        return 1
    except Exception as e:
        print(f"❌ Runtime Error: {e}")
        import traceback
        traceback.print_exc()
        print("\n🔧 Possible solutions:")
        print("   - Ensure pygame is installed: pip install pygame")
        print("   - Check file permissions")
        print("   - Verify map file format")
        print("   - Check import paths and directory structure")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)