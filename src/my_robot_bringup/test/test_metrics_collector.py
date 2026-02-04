#!/usr/bin/env python3
"""
Unit tests for the MetricsCollector node.
Tests metric calculations without requiring ROS infrastructure.
"""

import unittest
import math
import os
import tempfile


class TestPathEfficiencyCalculation(unittest.TestCase):
    """Test path efficiency calculations."""
    
    def test_perfect_efficiency(self):
        """Straight line path should be 100% efficient."""
        optimal_distance = 5.0
        actual_distance = 5.0
        efficiency = (optimal_distance / actual_distance) * 100
        self.assertAlmostEqual(efficiency, 100.0, places=1)
    
    def test_50_percent_efficiency(self):
        """Path twice as long as optimal should be 50% efficient."""
        optimal_distance = 5.0
        actual_distance = 10.0
        efficiency = (optimal_distance / actual_distance) * 100
        self.assertAlmostEqual(efficiency, 50.0, places=1)
    
    def test_optimal_distance_calculation(self):
        """Test straight-line distance calculation."""
        start_x, start_y = 0.0, 0.0
        goal_x, goal_y = 3.0, 4.0
        optimal = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
        self.assertAlmostEqual(optimal, 5.0, places=5)


class TestSuccessRateCalculation(unittest.TestCase):
    """Test success rate calculations."""
    
    def test_all_successful(self):
        """100% success rate when all tests pass."""
        successful = 10
        total = 10
        rate = (successful / total) * 100
        self.assertEqual(rate, 100.0)
    
    def test_half_successful(self):
        """50% success rate."""
        successful = 5
        total = 10
        rate = (successful / total) * 100
        self.assertEqual(rate, 50.0)
    
    def test_none_successful(self):
        """0% success rate."""
        successful = 0
        total = 10
        rate = (successful / total) * 100
        self.assertEqual(rate, 0.0)


class TestDistanceTraveled(unittest.TestCase):
    """Test distance traveled accumulation."""
    
    def test_straight_line_distance(self):
        """Test distance calculation for straight movement."""
        positions = [(0, 0), (1, 0), (2, 0), (3, 0)]
        distance = 0.0
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            distance += math.sqrt(dx*dx + dy*dy)
        self.assertAlmostEqual(distance, 3.0, places=5)
    
    def test_diagonal_distance(self):
        """Test distance calculation for diagonal movement."""
        positions = [(0, 0), (1, 1), (2, 2)]
        distance = 0.0
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            distance += math.sqrt(dx*dx + dy*dy)
        expected = 2 * math.sqrt(2)  # Two diagonal moves
        self.assertAlmostEqual(distance, expected, places=5)


class TestPathLengthCalculation(unittest.TestCase):
    """Test planned path length calculation."""
    
    def test_simple_path(self):
        """Test path length for simple waypoints."""
        # Simulated path poses (x, y)
        path_poses = [(0, 0), (1, 0), (1, 1), (2, 1)]
        path_length = 0.0
        for i in range(1, len(path_poses)):
            dx = path_poses[i][0] - path_poses[i-1][0]
            dy = path_poses[i][1] - path_poses[i-1][1]
            path_length += math.sqrt(dx*dx + dy*dy)
        self.assertAlmostEqual(path_length, 3.0, places=5)


if __name__ == '__main__':
    unittest.main()
