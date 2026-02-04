#!/usr/bin/env python3
"""
Unit tests for the MetricsCollector node.
Tests the actual calculation functions from metrics_collector.py.
"""

import unittest
import sys
import os

# Add scripts directory to path so we can import metrics_collector
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from scripts.metrics_collector import (
    calculate_distance,
    calculate_path_length,
    calculate_path_efficiency,
    calculate_success_rate,
    calculate_average
)


class TestCalculateDistance(unittest.TestCase):
    """Test the calculate_distance function."""
    
    def test_horizontal_distance(self):
        """Test horizontal distance calculation."""
        result = calculate_distance(0, 0, 3, 0)
        self.assertAlmostEqual(result, 3.0, places=5)
    
    def test_vertical_distance(self):
        """Test vertical distance calculation."""
        result = calculate_distance(0, 0, 0, 4)
        self.assertAlmostEqual(result, 4.0, places=5)
    
    def test_diagonal_distance(self):
        """Test 3-4-5 right triangle."""
        result = calculate_distance(0, 0, 3, 4)
        self.assertAlmostEqual(result, 5.0, places=5)
    
    def test_same_point(self):
        """Distance to same point should be zero."""
        result = calculate_distance(5, 5, 5, 5)
        self.assertAlmostEqual(result, 0.0, places=5)
    
    def test_negative_coordinates(self):
        """Test with negative coordinates."""
        result = calculate_distance(-1, -1, 2, 3)
        expected = 5.0  # 3-4-5 triangle
        self.assertAlmostEqual(result, expected, places=5)


class TestCalculatePathLength(unittest.TestCase):
    """Test the calculate_path_length function."""
    
    def test_straight_line_path(self):
        """Test distance for straight movement."""
        positions = [(0, 0), (1, 0), (2, 0), (3, 0)]
        result = calculate_path_length(positions)
        self.assertAlmostEqual(result, 3.0, places=5)
    
    def test_diagonal_path(self):
        """Test distance for diagonal movement."""
        positions = [(0, 0), (1, 1), (2, 2)]
        result = calculate_path_length(positions)
        expected = 2 * (2 ** 0.5)  # Two diagonal moves
        self.assertAlmostEqual(result, expected, places=5)
    
    def test_l_shaped_path(self):
        """Test L-shaped path."""
        positions = [(0, 0), (3, 0), (3, 4)]
        result = calculate_path_length(positions)
        self.assertAlmostEqual(result, 7.0, places=5)
    
    def test_single_point(self):
        """Single point should have zero length."""
        result = calculate_path_length([(0, 0)])
        self.assertAlmostEqual(result, 0.0, places=5)
    
    def test_empty_path(self):
        """Empty path should have zero length."""
        result = calculate_path_length([])
        self.assertAlmostEqual(result, 0.0, places=5)


class TestCalculatePathEfficiency(unittest.TestCase):
    """Test the calculate_path_efficiency function."""
    
    def test_perfect_efficiency(self):
        """Straight line path should be 100% efficient."""
        result = calculate_path_efficiency(5.0, 5.0)
        self.assertAlmostEqual(result, 100.0, places=1)
    
    def test_50_percent_efficiency(self):
        """Path twice as long as optimal should be 50% efficient."""
        result = calculate_path_efficiency(5.0, 10.0)
        self.assertAlmostEqual(result, 50.0, places=1)
    
    def test_25_percent_efficiency(self):
        """Path 4x as long as optimal should be 25% efficient."""
        result = calculate_path_efficiency(5.0, 20.0)
        self.assertAlmostEqual(result, 25.0, places=1)
    
    def test_zero_actual_distance(self):
        """Zero actual distance should return 0% efficiency."""
        result = calculate_path_efficiency(5.0, 0.0)
        self.assertAlmostEqual(result, 0.0, places=1)
    
    def test_negative_actual_distance(self):
        """Negative actual distance should return 0% efficiency."""
        result = calculate_path_efficiency(5.0, -1.0)
        self.assertAlmostEqual(result, 0.0, places=1)


class TestCalculateSuccessRate(unittest.TestCase):
    """Test the calculate_success_rate function."""
    
    def test_all_successful(self):
        """100% success rate when all tests pass."""
        result = calculate_success_rate(10, 10)
        self.assertEqual(result, 100.0)
    
    def test_half_successful(self):
        """50% success rate."""
        result = calculate_success_rate(5, 10)
        self.assertEqual(result, 50.0)
    
    def test_none_successful(self):
        """0% success rate."""
        result = calculate_success_rate(0, 10)
        self.assertEqual(result, 0.0)
    
    def test_zero_total(self):
        """Zero total tests should return 0%."""
        result = calculate_success_rate(0, 0)
        self.assertEqual(result, 0.0)
    
    def test_negative_total(self):
        """Negative total should return 0%."""
        result = calculate_success_rate(5, -1)
        self.assertEqual(result, 0.0)


class TestCalculateAverage(unittest.TestCase):
    """Test the calculate_average function."""
    
    def test_simple_average(self):
        """Test simple average calculation."""
        result = calculate_average([10, 20, 30])
        self.assertAlmostEqual(result, 20.0, places=5)
    
    def test_single_value(self):
        """Single value average."""
        result = calculate_average([42])
        self.assertAlmostEqual(result, 42.0, places=5)
    
    def test_empty_list(self):
        """Empty list should return 0."""
        result = calculate_average([])
        self.assertAlmostEqual(result, 0.0, places=5)
    
    def test_decimal_average(self):
        """Test average with decimals."""
        result = calculate_average([1, 2, 3, 4])
        self.assertAlmostEqual(result, 2.5, places=5)


if __name__ == '__main__':
    unittest.main()
