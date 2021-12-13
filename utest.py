import unittest
from generator import Segment
import random
# Test cases to test Calulator methods
# You always create  a child class derived from unittest.TestCase


class TestGen(unittest.TestCase):

    def setUp(self):
        self.segment = Segment(3)

    def test_1(self):
        self.segment.current_position = [0.0, 0.0, 0.0]
        self.segment.target_position = [1.0, 2.0, 0.5]
        self.segment.current_velocity = [0.0, 0.0, 0.0]
        self.segment.target_velocity = [0.0, 0.0, 0.0]

        self.segment.max_velocity = [1.2, 1.2, 1.2]
        self.segment.max_acceleration = [1.8, 1.8, 1.8]
        self.segment.max_jerk = [1.9, 1.9, 1.9]
        self.segment.calculate()
        for i in range(self.segment.dof):
            if i == self.segment.dof - 1:
                break
            else:
                self.assertAlmostEqual(
                    sum(self.segment.time_stamp[i]), sum(self.segment.time_stamp[i+1]))
                self.assertAlmostEqual(
                    sum(self.segment.time_stamp[i][:3]), sum(self.segment.time_stamp[i+1][:3]))
                self.assertAlmostEqual(
                    sum(self.segment.time_stamp[i][4:]), sum(self.segment.time_stamp[i+1][4:]))

        # self.assertEqual(1, 1)

    def test_2(self):
        self.segment.current_position = [
            random.random() * 3 - 3, random.random() * 3 - 3, random.random() * 3 - 3]
        self.segment.target_position = [
            random.random() * 3 - 3, random.random() * 3 - 3, random.random() * 3 - 3]
        self.segment.current_velocity = [
            random.random() * 3 - 3, random.random() * 3 - 3, random.random() * 3 - 3]
        self.segment.target_velocity = [
            random.random() * 3 - 3, random.random() * 3 - 3, random.random() * 3 - 3]
        self.segment.max_velocity = [2, 2, 2]
        self.segment.max_acceleration = [2, 2, 2]
        self.segment.max_jerk = [
            2, 2, 2]
        self.segment.calculate()
        for i in range(self.segment.dof):
            if i == self.segment.dof - 1:
                break
            else:
                self.assertAlmostEqual(
                    sum(self.segment.time_stamp[i]), sum(self.segment.time_stamp[i+1]), delta=0.0001)
                self.assertAlmostEqual(
                    sum(self.segment.time_stamp[i][:3]), sum(self.segment.time_stamp[i+1][:3]), delta=0.0001)
                self.assertAlmostEqual(
                    sum(self.segment.time_stamp[i][4:]), sum(self.segment.time_stamp[i+1][4:]), delta=0.0001)

        # self.assertEqual(1, 1)

    def test_3(self):
        self.segment.current_position = [
            random.random() * 3 - 3, random.random() * 3 - 3, random.random() * 3 - 3]
        self.segment.target_position = [
            random.random() * 3 - 3, random.random() * 3 - 3, random.random() * 3 - 3]
        self.segment.current_velocity = [
            random.random() * 3 - 3, random.random() * 3 - 3, random.random() * 3 - 3]
        self.segment.target_velocity = [
            random.random() * 3 - 3, random.random() * 3 - 3, random.random() * 3 - 3]
        self.segment.max_velocity = [2, 2, 2]
        self.segment.max_acceleration = [2, 2, 2]
        self.segment.max_jerk = [
            2, 2, 2]
        self.segment.calculate()

        for i in range(self.segment.dof):
            if i == self.segment.dof - 1:
                break
            else:
                self.assertAlmostEqual(
                    sum(self.segment.time_stamp[i]), sum(self.segment.time_stamp[i+1]), delta=0.0001)
                self.assertAlmostEqual(
                    sum(self.segment.time_stamp[i][:3]), sum(self.segment.time_stamp[i+1][:3]), delta=0.0001)
                self.assertAlmostEqual(
                    sum(self.segment.time_stamp[i][4:]), sum(self.segment.time_stamp[i+1][4:]), delta=0.0001)

        # self.assertEqual(1, 1)


if __name__ == "__main__":

    unittest.main()
