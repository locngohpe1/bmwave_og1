import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def main():
    print("🚀 LAUNCHING ε⋆+ ALGORITHM")
     # Import and run the sensor-based algorithm
    from epsilon_star.runner import main as run_sensor_algorithm
    run_sensor_algorithm()

if __name__ == "__main__":
    main()