#from ruckig import Reflexxes
from ruckig import InputParameter, OutputParameter, Result, Trajectory, Ruckig, Synchronization
from copy import copy
from pathlib import Path
from sys import path

import numpy as np

from plotter import Plotter

path.insert(0, str(Path(__file__).parent.parent / 'trajgenerator-ruckig'))


class Traj:
    def __init__(self, otg, inp, res):
        self.otg = otg
        self.inp = inp
        self.t_list = res[0]
        self.out_list = res[1]


def walk_through_trajectory(otg, inp, start_time=0):
    t_list, out_list = [], []
    out = OutputParameter(inp.degrees_of_freedom)

    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)

        inp.current_position = out.new_position
        inp.current_velocity = out.new_velocity
        inp.current_acceleration = out.new_acceleration

        t_list.append(out.time + start_time)
        out_list.append(copy(out))

    return t_list, out_list


def calc_segment(prev, current_pos, target_pos, max_velocity, max_acceleration, max_jerk, start_time, is_last, dir):
    calc = InputParameter(len(current_pos))
    calc.current_position = current_pos
    calc.target_position = target_pos
    calc.max_velocity = [max_velocity] * len(current_pos)
    # if not is_last:
    #     calc.target_velocity = calc.max_velocity
    calc.max_acceleration = [max_acceleration] * len(current_pos)
    calc.max_jerk = [max_jerk] * len(current_pos)
    calc.synchronization = Synchronization.Phase

    if prev is not None:
        calc.current_velocity = prev.out_list[-1].new_velocity
        calc.current_acceleration = prev.out_list[-1].new_acceleration

    # for index in range(len(dir)):
    #     if dir[index] == 0:
    #         calc.current_velocity[index] = 0
    #         calc.current_acceleration[index] = 0
    #     else:
    #         calc.target_velocity[index] = prev.out_list[-1].new_velocity[index]
    #         calc.current_acceleration[index] = prev.out_list[-1].new_acceleration[index]

    otg = Ruckig(calc.degrees_of_freedom, 0.1)
    out = Trajectory(len(current_pos))
    res = otg.calculate(calc, out)
    max_vel_point = out.at_time(out.duration / 2.0)

    index = 0
    v = []
    for d in dir:
        if d == 0 or is_last:
            v.append(0)
        else:
            v.append(max_vel_point[1][index] * 1.5)

        index = index + 1

    calc.target_velocity = v
    # if not is_last:
    #     v = []
    #     for el in max_vel_point[1]:
    #         v.append(el)
    #
    #     calc.target_velocity = v

    otg = Ruckig(calc.degrees_of_freedom, 0.1)
    return Traj(otg, calc, walk_through_trajectory(otg, calc, start_time))


def calc_multisegment(points, max_velocity, max_acceleration, max_jerk):
    index = 0
    res = []
    start_time = 0
    dir = []
    while index < (len(points) - 1):
        p1 = points[index + 0]
        p2 = points[index + 1]
        dir.append(np.sign(np.subtract(p2, p1)))
        index = index + 1

    dir.append(0)
    index = 0
    while index < (len(points) - 1):
        p1 = points[index + 0]
        p2 = points[index + 1]
        d = np.add(dir[index + 0], dir[index + 1])
        is_last = (index + 1) == (len(points) - 1)
        res.append(calc_segment(res[-1] if len(res) > 0 else None, p1, p2, max_velocity, max_acceleration, max_jerk,
                                start_time, is_last, d))
        start_time = res[-1].t_list[-1]
        index = index + 1

    return res


if __name__ == '__main__':
    inp = InputParameter(3)
    # inp.interface = Interface.Velocity
    inp.synchronization = Synchronization.Phase
    # inp.duration_discretization = DurationDiscretization.Discrete

    points = [[0, 0, 0], [1, -1, 1], [2, 0, 2], [1, -2, 0]]
    max_velocity = 2
    max_acceleration = 2
    max_jerk = 2

    res = calc_multisegment(points, max_velocity, max_acceleration, max_jerk)

    # print(f'Calculation duration: {out_list[0].calculation_duration:0.1f} [µs]')
    # print(f'Trajectory duration: {out_list[0].trajectory.duration:0.4f} [s]')

    Plotter.plot_trajectory('otg_trajectory.png', res, True)
