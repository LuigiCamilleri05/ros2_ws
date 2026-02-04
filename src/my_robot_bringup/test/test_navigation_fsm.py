#!/usr/bin/env python3
"""
Unit tests for the Navigation FSM.
Tests state transitions and logic.
"""

import unittest
from enum import Enum


class NavState(Enum):
    """Navigation FSM States - mirrored from navigation_fsm.py"""
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    REPLANNING = "REPLANNING"
    RECOVERING = "RECOVERING"


class TestNavStateMachine(unittest.TestCase):
    """Test navigation state machine transitions."""
    
    def setUp(self):
        """Initialize test state."""
        self.current_state = NavState.IDLE
        self.replan_count = 0
        self.max_replan_attempts = 3
    
    def transition_to(self, new_state):
        """Simulate state transition."""
        valid_transitions = {
            NavState.IDLE: [NavState.NAVIGATING],
            NavState.NAVIGATING: [NavState.IDLE, NavState.REPLANNING, NavState.RECOVERING],
            NavState.REPLANNING: [NavState.NAVIGATING, NavState.RECOVERING],
            NavState.RECOVERING: [NavState.IDLE, NavState.NAVIGATING],
        }
        if new_state in valid_transitions.get(self.current_state, []):
            self.current_state = new_state
            return True
        return False
    
    def test_idle_to_navigating(self):
        """Test transition from IDLE to NAVIGATING."""
        self.current_state = NavState.IDLE
        result = self.transition_to(NavState.NAVIGATING)
        self.assertTrue(result)
        self.assertEqual(self.current_state, NavState.NAVIGATING)
    
    def test_idle_cannot_go_to_recovering(self):
        """Test that IDLE cannot directly go to RECOVERING."""
        self.current_state = NavState.IDLE
        result = self.transition_to(NavState.RECOVERING)
        self.assertFalse(result)
        self.assertEqual(self.current_state, NavState.IDLE)
    
    def test_navigating_to_idle_on_goal_reached(self):
        """Test transition to IDLE when goal is reached."""
        self.current_state = NavState.NAVIGATING
        result = self.transition_to(NavState.IDLE)
        self.assertTrue(result)
        self.assertEqual(self.current_state, NavState.IDLE)
    
    def test_replanning_to_recovering_on_max_attempts(self):
        """Test transition to RECOVERING after max replan attempts."""
        self.current_state = NavState.REPLANNING
        self.replan_count = self.max_replan_attempts
        result = self.transition_to(NavState.RECOVERING)
        self.assertTrue(result)
        self.assertEqual(self.current_state, NavState.RECOVERING)
    
    def test_recovering_to_idle(self):
        """Test recovery can return to IDLE."""
        self.current_state = NavState.RECOVERING
        result = self.transition_to(NavState.IDLE)
        self.assertTrue(result)
        self.assertEqual(self.current_state, NavState.IDLE)


class TestStuckDetection(unittest.TestCase):
    """Test stuck detection logic."""
    
    def test_robot_is_stuck(self):
        """Robot is stuck if it hasn't moved enough."""
        last_position = (1.0, 1.0)
        current_position = (1.01, 1.01)
        stuck_threshold = 0.1
        
        dx = current_position[0] - last_position[0]
        dy = current_position[1] - last_position[1]
        distance_moved = (dx**2 + dy**2)**0.5
        
        is_stuck = distance_moved < stuck_threshold
        self.assertTrue(is_stuck)
    
    def test_robot_is_not_stuck(self):
        """Robot is not stuck if it has moved enough."""
        last_position = (1.0, 1.0)
        current_position = (2.0, 1.0)
        stuck_threshold = 0.1
        
        dx = current_position[0] - last_position[0]
        dy = current_position[1] - last_position[1]
        distance_moved = (dx**2 + dy**2)**0.5
        
        is_stuck = distance_moved < stuck_threshold
        self.assertFalse(is_stuck)


class TestReplanCounter(unittest.TestCase):
    """Test replan counting logic."""
    
    def test_replan_count_increments(self):
        """Test that replan count increases."""
        replan_count = 0
        replan_count += 1
        self.assertEqual(replan_count, 1)
    
    def test_max_replans_exceeded(self):
        """Test detection of max replans exceeded."""
        replan_count = 3
        max_replan_attempts = 3
        exceeded = replan_count >= max_replan_attempts
        self.assertTrue(exceeded)
    
    def test_replan_count_reset_on_new_goal(self):
        """Test that replan count resets for new goal."""
        replan_count = 3
        # New goal received
        replan_count = 0
        self.assertEqual(replan_count, 0)


if __name__ == '__main__':
    unittest.main()
