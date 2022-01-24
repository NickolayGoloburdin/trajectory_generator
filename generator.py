from math import sqrt
from time import time
import matplotlib.pyplot as plt
from numpy import float32, ndarray, sign
import numpy as np
from scipy import optimize
from scipy.optimize import fsolve, least_squares
import copy


def plot(segments):
    time = []
    positions = []
    velocity = []
    acceleration = []
    for _ in range(len(segments[0].current_position)):
        positions.append([])
        velocity.append([])
        acceleration.append([])

    time_step = 0.01
    time_offset = 0
    for i, segment in enumerate(segments):
        time.append(time_offset)
        while time[-1] < (time_offset+segment.duration):
            data = segment.at_time(time[-1]-time_offset)
            for i, _ in enumerate(data[0]):
                positions[i].append(data[0][i])
                velocity[i].append(data[1][i])
                acceleration[i].append(data[2][i])
            time.append(time[-1]+time_step)
        time.pop()
        time_offset = time[-1]
    fig, axs = plt.subplots(3, 1, figsize=(12, 7))
    axs[0].plot(time, positions[0])
    axs[0].plot(time, velocity[0])
    axs[0].plot(time, acceleration[0])
    axs[0].grid()

    axs[1].plot(time, positions[1])
    axs[1].plot(time, velocity[1])
    axs[1].plot(time, acceleration[1])
    axs[1].grid()

    axs[2].plot(time, positions[2])
    axs[2].plot(time, velocity[2])
    axs[2].plot(time, acceleration[2])
    axs[2].grid()
    plt.show()


class Segment():
    current_position = []
    target_position = []
    current_velocity = []
    target_velocity = []

    max_velocity = []
    max_acceleration = []
    max_jerk = []

    time_stamp = []
    state_vectors = []

    duration = 0

    def __init__(self, dof) -> None:
        self.dof = dof
        self.current_position = [0.0]*dof
        self.target_position = [0.0]*dof
        self.current_velocity = [0.0]*dof
        self.target_velocity = [0.0]*dof

    def calculate(self):
        self.calculate_time_stamp()
        self.synchronization()
        self.state_vectors_generate()
        self.calculate_duration()

    def erase_segment(self):
        self.__init__()
        current_position = []
        target_position = []
        current_velocity = []
        target_velocity = []

        max_velocity = []
        max_acceleration = []
        max_jerk = []

        time_stamp = []
        state_vectors = []

        duration = 0

    def at_time(self, time):
        positions = []
        velocities = []
        accelerations = []
        jerks = []

        for i in range(self.dof):
            coords = self.__get_joint_states(i, time)
            positions.append(coords[0])
            velocities.append(coords[1])
            accelerations.append(coords[2])
            jerks.append(coords[3])

        return [positions, velocities, accelerations, jerks]

    def calculate_time_stamp(self) -> None:
        for i in range(self.dof):

            times, _ = self.get_time_stamp(self.max_velocity[i], index=i)
            if times[3] < 0:
                start_point = [self.max_velocity[i]/2]
                res1 = optimize.fsolve(
                    self.equation_minimize, start_point, args=(i,), full_output=True)
                times, _ = self.get_time_stamp(res1[0][0], index=i)
            self.time_stamp.append(times)

    def equation_minimize(self, v_, *args):
        v = v_[0]
        index = args[0]
        dq = self.target_position[index] - self.current_position[index]

        _, q_r_s = self.get_time_stamp(v, index)

        res = (q_r_s-dq)

        return [res]

    def get_time_stamp(self, v, index):
        dq = self.target_position[index] - self.current_position[index]

        direction = dq/abs(dq)

        dv_cur = direction*v - self.current_velocity[index]
        if abs(dv_cur) <= self.max_acceleration[index]**2/self.max_jerk[index]:
            t1 = t3 = sqrt(abs(dv_cur)/self.max_jerk[index])
            t2 = 0
            q1_3 = t1*(dv_cur+2*self.current_velocity[index])
        else:
            t1 = t3 = self.max_acceleration[index]/self.max_jerk[index]
            t2 = abs(dv_cur)/self.max_acceleration[index] - \
                self.max_acceleration[index]/self.max_jerk[index]
            q1_3 = 1/2*(2*t1+t2)*(direction*v+self.current_velocity[index])

        dv_tar = direction*v - self.target_velocity[index]
        if abs(dv_tar) <= self.max_acceleration[index]**2/self.max_jerk[index]:
            t5 = t7 = sqrt(abs(dv_tar)/self.max_jerk[index])
            t6 = 0
            q5_7 = t5*(dv_tar+2*self.target_velocity[index])
        else:
            t5 = t7 = self.max_acceleration[index]/self.max_jerk[index]
            t6 = abs(dv_tar)/self.max_acceleration[index] - \
                self.max_acceleration[index]/self.max_jerk[index]
            q5_7 = 1/2*(2*t5+t6)*(direction*v+self.target_velocity[index])

        t4 = direction*(dq - (q1_3+q5_7))/v
        return [t1, t2, t3, t4, t5, t6, t7], q1_3+q5_7

    def calculate_duration(self):
        duration = sum(self.time_stamp[0])
        self.duration = duration
        return duration, self.time_stamp

    def get_joint_states(self, i, time):
        state_vector = self.state_vectors[i]
        time_joint = [sum(self.time_stamp[i][0:j+1])
                      for j, el in enumerate(self.time_stamp[i])]
        time_joint.insert(0, 0)

        def get_i(t):
            pose = -1
            for i, el in enumerate(time_joint[1:]):
                if t <= el:
                    pose = i
                    break
                else:
                    continue

            return pose

        index = get_i(time)
        q = state_vector[index][3] + \
            state_vector[index][2]*(time - time_joint[index]) + \
            state_vector[index][1]*(time - time_joint[index])**2/2 + \
            state_vector[index][0]*(time - time_joint[index])**3/6

        v = state_vector[index][2] + \
            state_vector[index][1]*(time - time_joint[index]) + \
            state_vector[index][0]*(time - time_joint[index])**2/2

        a = state_vector[index][1] + \
            state_vector[index][0]*(time - time_joint[index])

        j = state_vector[index][0]

        return [q, v, a, j]

    def state_vectors_generate(self):
        for i in range(self.dof):
            t = self.time_stamp[i]
            vmax = self.max_velocity[i]
            jmax = self.max_jerk[i]

            q1 = self.current_position[i]
            q2 = self.target_position[i]

            vin = self.current_velocity[i]

            sign = abs(q2 - q1)/(q2 - q1)

            state_vector = []

            for k in range(7):
                if k in [1, 3, 5]:
                    j = 0
                elif k == 0:
                    j = sign*jmax
                elif k == 2:
                    j = -sign*jmax
                elif k == 4:
                    j = -sign*jmax
                else:
                    j = sign*jmax

                if k == 0:
                    a = 0
                    v = vin
                    q = q1
                else:
                    a = state_vector[k-1][1] + state_vector[k-1][0]*t[k-1]
                    v = state_vector[k-1][2] + state_vector[k-1][1] * \
                        t[k-1] + 1/2*state_vector[k-1][0]*t[k-1]**2
                    q = state_vector[k-1][3] + state_vector[k-1][2]*t[k-1] + 1/2 * \
                        state_vector[k-1][1]*t[k-1]**2 + 1 / \
                        6*state_vector[k-1][0]*t[k-1]**3
                state_vector.append([j, a, v, q])

            self.state_vectors.append(state_vector)

    def get_index_base_motor(self):
        index = 0
        max_time = sum(self.time_stamp[0])
        for i, el in enumerate(self.time_stamp):
            if max_time < sum(el):
                max_time = sum(el)
                index = i

        return index

    def modify_motor_times_parameter(self, index, base_index):

        t_1_3 = sum(self.time_stamp[base_index][:3])
        t_5_7 = sum(self.time_stamp[base_index][4:])
        t4 = self.time_stamp[base_index][3]
        dq = self.target_position[index] - self.current_position[index]

        v = (dq - 1/2*(t_1_3*self.current_velocity[index] + t_5_7 *
                       self.target_velocity[index]))/(1/2*t_1_3+1/2*t_5_7+t4)

        dv_cur = v - self.current_velocity[index]
        dv_tar = v - self.target_velocity[index]

        t_2_2 = (t_1_3**2*self.max_jerk[index] -
                 4*abs(dv_cur))/self.max_jerk[index]
        if t_2_2 > 0:
            t2 = sqrt(t_2_2)

        else:
            t2 = 0

        t1 = 1/2*t_1_3 - 1/2*t2

        t_6_2 = (t_5_7**2*self.max_jerk[index] -
                 4*abs(dv_tar))/self.max_jerk[index]
        if t_6_2 > 0:
            t6 = sqrt(t_6_2)
        else:
            t6 = 0

        t5 = 1/2*t_5_7 - 1/2*t6

        new_time_stamp = [t1, t2, t1, t4, t5, t6, t5]
        self.time_stamp[index] = new_time_stamp

    def synchronization(self):
        base_index = self.get_index_base_motor()
        for i in range(self.dof):
            if i == base_index:
                continue
            else:
                self.modify_motor_times_parameter(i, base_index)


def two_segment(segment, intermediate_waypoint):
    args_ = [segment, intermediate_waypoint]
    intermediate_velocities = []
    lower_bound = []
    for i in segment.max_velocity:
        intermediate_velocities.append(i/2)
    intermediate_velocities_array = np.array(intermediate_velocities)

    # intermediate_velocities, dtype=np.float32)

    intermediate_velocities = least_squares(
        equationmultiple, intermediate_velocities, bounds=((-5.0, -5.0, -5.0), (5.0, 5.0, 5.0)), args=args_).x
    first_segment = copy.deepcopy(segment)
    second_segment = copy.deepcopy(segment)
    first_segment.target_velocity = second_segment.current_velocity = intermediate_velocities
    first_segment.target_position = second_segment.current_position = intermediate_waypoint
    first_segment.calculate()
    second_segment.calculate()
    return [first_segment, second_segment]


def sign(a, b):
    if (b - a < 0):
        return - 1
    elif (b - a == 0):
        return 0
    else:
        return 1


def equationmultiple(vars, *args):
    variable_velocity = vars
    [segment, intermediate_waypoint] = args
    first_segment = copy.deepcopy(segment)
    second_segment = copy.deepcopy(segment)
    first_segment.target_position = intermediate_waypoint
    second_segment.current_position = intermediate_waypoint
    for i in range(segment.dof):
        if sign(first_segment.current_position[i], intermediate_waypoint[i]) != sign(first_segment.current_position[i], intermediate_waypoint[i]):
            first_segment.target_velocity[i] = 0.0
        else:
            first_segment.target_velocity[i] = variable_velocity[i]
    first_segment.calculate()
    second_segment.current_velocity = first_segment.target_velocity
    second_segment.calculate()
    return first_segment.calculate_duration() + second_segment.calculate_duration()


segment = Segment(3)

segment.current_position = [0.0, 0.0, 0.0]
segment.target_position = [1.0, 2.0, 0.5]
segment.current_velocity = [0.0, 0.0, 0.0]
segment.target_velocity = [0.0, 0.0, 0.0]

segment.max_velocity = [1.2, 1.2, 1.2]
segment.max_acceleration = [1.8, 1.8, 1.8]
segment.max_jerk = [1.9, 1.9, 1.9]
intermediate_waypoint = [2.0, 1.0, 2.0]
start_time = time()
segment1, segment2 = two_segment(segment, intermediate_waypoint)


print(time()-start_time)

# plot([segment])
