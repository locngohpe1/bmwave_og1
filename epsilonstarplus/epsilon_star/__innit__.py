"""
epsilon_star/__init__.py
Package initialization cho ε⋆+ Algorithm

Exports main classes và functions cho external use
"""

from .core import (
    Position,
    EnergyConfig,
    ETMState,
    CellState,
    SegmentType,
    MAPSHierarchy
)

from .etm import ExploratoryTuringMachine

from .robot import (
    EpsilonStarPlusRobot,
    VisibilityGraph,
    a_star_search
)

from .runner import (
    EpsilonStarPlusRunner,
    run_quick_demo
)

__version__ = "2.0.0"
__author__ = "Implementation based on Song & Gupta (2018), Shen et al. (2020)"

__all__ = [
    # Core classes
    'Position',
    'EnergyConfig',
    'ETMState',
    'CellState',
    'SegmentType',
    'MAPSHierarchy',

    # ETM
    'ExploratoryTuringMachine',

    # Main robot
    'EpsilonStarPlusRobot',
    'VisibilityGraph',
    'a_star_search',

    # Runner
    'EpsilonStarPlusRunner',
    'run_quick_demo'
]


def create_test_environment(rows: int, cols: int) -> 'np.ndarray':
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


def run_simple_test():
    """Run a simple test of the ε⋆+ algorithm"""
    print("Running simple ε⋆+ test...")

    # Create test environment
    environment = create_test_environment(15, 15)
    battery_pos = Position(0, 0)

    # Create robot
    robot = EpsilonStarPlusRobot(battery_pos, 15, 15)
    robot.set_environment(environment)

    # Run test
    steps = 0
    while not robot.is_complete() and steps < 300:
        robot.run_step()
        steps += 1

    # Print results
    stats = robot.get_statistics()
    print(f"Test completed in {steps} steps")
    print(f"Coverage: {stats['coverage_percentage']:.1f}%")
    print(f"Path length: {stats['total_path_length']:.1f}")
    print(f"Returns: {stats['return_count']}")


if __name__ == "__main__":
    run_simple_test()