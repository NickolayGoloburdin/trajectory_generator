import unittest
from generator import Segment
import random
# Test cases to test Calulator methods
# You always create  a child class derived from unittest.TestCase


class TestGen(unittest.TestCase):
    # setUp method is overridden from the parent class TestCase
    def setUp(self):
        self.segment = Segment(3)
    # Each test method starts with the keyword test_

    def test_1(self):
        self.segment.current_position = [0.0, 0.0, 0.0]
        self.segment.target_position = [1.0, 2.0, 0.5]
        self.segment.current_velocity = [0.0, 0.0, 0.0]
        self.segment.target_velocity = [0.0, 0.0, 0.0]

        self.segment.max_velocity = [1.2, 1.2, 1.2]
        self.segment.max_acceleration = [1.8, 1.8, 1.8]
        self.segment.max_jerk = [1.9, 1.9, 1.9]
        self.segment.calculate()
        self.assertEqual(
            sum(self.segment.time_stamp[0][:3]), sum(self.segment.time_stamp[1][:3]), sum(self.segment.time_stamp[2][:3]))
        self.assertEqual(
            sum(self.segment.time_stamp[0][4:]), sum(self.segment.time_stamp[1][4:]), sum(self.segment.time_stamp[2][4:]))

        # self.assertEqual(1, 1)

    def test_2(self):
        self.segment.current_position = [
            random.random() * 3.0-3, random.random() * 3 - 3, random.random() * 3 - 3]
        self.segment.target_position = [
            random.random() * 3.0-3, random.random() * 3 - 3, random.random() * 3-3]
        self.segment.current_velocity = [
            random.random() * 3.0-3, random.random() * 3-3, random.random() * 3-3]
        self.segment.target_velocity = [
            random.random() * 3.0-3, random.random() * 3-3, random.random() * 3-3]
        self.segment.max_velocity = [2, 2, 2]
        self.segment.max_acceleration = [2, 2, 2]
        self.segment.max_jerk = [
            2, 2, 2]
        self.segment.calculate()
        self.assertEqual(
            sum(self.segment.time_stamp[0][:3]), sum(self.segment.time_stamp[1][:3]), sum(self.segment.time_stamp[2][:3]))
        self.assertEqual(
            sum(self.segment.time_stamp[0][4:]), sum(self.segment.time_stamp[1][4:]), sum(self.segment.time_stamp[2][4:]))

        # self.assertEqual(1, 1)

    def test_3(self):
        self.segment.current_position = [
            random.random() * 3.0-3, random.random() * 3 - 3, random.random() * 3 - 3]
        self.segment.target_position = [
            random.random() * 3.0-3, random.random() * 3 - 3, random.random() * 3-3]
        self.segment.current_velocity = [
            random.random() * 3.0-3, random.random() * 3-3, random.random() * 3-3]
        self.segment.target_velocity = [
            random.random() * 3.0-3, random.random() * 3-3, random.random() * 3-3]
        self.segment.max_velocity = [2, 2, 2]
        self.segment.max_acceleration = [2, 2, 2]
        self.segment.max_jerk = [
            2, 2, 2]
        self.segment.calculate()
        self.assertEqual(
            sum(self.segment.time_stamp[0][:3]), sum(self.segment.time_stamp[1][:3]), sum(self.segment.time_stamp[2][:3]))
        self.assertEqual(
            sum(self.segment.time_stamp[0][4:]), sum(self.segment.time_stamp[1][4:]), sum(self.segment.time_stamp[2][4:]))

        # self.assertEqual(1, 1)


if __name__ == "__main__":

    unittest.main()
