from ruckig import OutputParameter, Result
from copy import copy
from pathlib import Path
from sys import path

import matplotlib.pyplot as plt
import numpy as np

path.insert(0, str(Path(__file__).parent.parent / 'trajgenerator-ruckig'))


class Plotter:
    @staticmethod
    def plot_trajectory(filename, res, show=False):
        qaxis = None
        dqaxis = None
        ddqaxis = None
        dddqaxis = None
        degrees_of_freedom = res[0].out_list[0].degrees_of_freedom
        t = []
        for el in res:
            t = np.concatenate([t, el.t_list])
            tr = el.out_list
            arr1 = np.array(list(map(lambda x: x.new_position, tr)))
            if qaxis is None:
                qaxis = arr1
            else:
                qaxis = np.concatenate([qaxis, arr1])

            arr2 = np.array(list(map(lambda x: x.new_velocity, tr)))
            if dqaxis is None:
                dqaxis = arr2
            else:
                dqaxis = np.concatenate([dqaxis, arr2])

            arr3 = np.array(list(map(lambda x: x.new_acceleration, tr)))
            if ddqaxis is None:
                ddqaxis = arr3
            else:
                ddqaxis = np.concatenate([ddqaxis, arr3])

        plt.figure(figsize=(8.0, 2.0 + 3.0 * degrees_of_freedom), dpi=120)

        for dof in range(degrees_of_freedom):

            plt.subplot(degrees_of_freedom, 1, dof + 1)
            plt.plot(t, qaxis[:, dof], label=f'Position {dof+1}')
            plt.plot(t, dqaxis[:, dof], label=f'Velocity {dof+1}')
            plt.plot(t, ddqaxis[:, dof], label=f'Acceleration {dof+1}')
            # plt.plot(t_list, dddqaxis[:, dof], label=f'Jerk {dof+1}')

            for el in res:
                t1 = el.t_list[0]
                t2 = el.t_list[-1]
                # plt.axvline(x=t1, color='orange', linestyle='--', linewidth=1.1)
                plt.axvline(x=t2, color='red', linestyle='--', linewidth=1.1)
                print(f"{t1} -> {t2}")

            plt.legend()
            plt.grid(True)

        plt.xlabel('t')
        plt.savefig(Path(__file__).parent.parent /
                    'trajgenerator-ruckig' / filename)

        if show:
            plt.show()
