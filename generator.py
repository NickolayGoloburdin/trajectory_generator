from math import sqrt
from time import time
import matplotlib.pyplot as plt
from scipy import optimize
from scipy.optimize import fsolve, least_squares, minimize
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

    time_step = 0.001
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
    fig, axs = plt.subplots(segments[0].dof, 1, figsize=(12, 7))
    if segments[0].dof == 1:
        axs.plot(time, positions[0])
        axs.plot(time, velocity[0])
        axs.plot(time, acceleration[0])
        axs.grid()
    else:
        for i in range(segments[0].dof):
            axs[i].plot(time, positions[i])
            axs[i].plot(time, velocity[i])
            axs[i].plot(time, acceleration[i])
            axs[i].grid()
    plt.show()


class Segment():

    def __init__(self, dof) -> None:
        self.dof = dof
        self.current_position = [0.0]*dof
        self.target_position = [0.0]*dof
        self.current_velocity = [0.0]*dof
        self.target_velocity = [0.0]*dof
        self.current_position = []
        self.target_position = []
        self.current_velocity = []
        self.target_velocity = []
        self.max_velocity = []
        self.real_max_velocity = [0.0]*dof
        self.max_acceleration = []
        self.max_jerk = []
        self.time_stamp = []
        self.motors_with_zero_dq = []
        self.state_vectors = []
        self.duration = 0

    def calculate(self):
        self.calculate_time_stamp()
        self.synchronization()
        self.state_vectors_generate()
        self.calculate_duration()

    def zero_vel_current(self):
        for i in self.current_velocity:
            i = 0.0

    def zero_vel_target(self):
        for i in self.target_velocity:
            i = 0.0

    def erase_segment(self):
        self.time_stamp.clear()
        self.state_vectors.clear()

        self.duration = 0

    def at_time(self, time):
        positions = []
        velocities = []
        accelerations = []
        jerks = []

        for i in range(self.dof):
            coords = self.get_joint_states(i, time)
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
                self.real_max_velocity[i] = res1[0][0]
                times, _ = self.get_time_stamp(res1[0][0], index=i)
            else:
                self.real_max_velocity[i] = self.max_velocity[i]
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

        if dq == 0:
            self.motors_with_zero_dq.append(index)
            return [0, 0, 0, 0, 0, 0, 0], 0

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
            jmax = self.max_jerk[i]

            q1 = self.current_position[i]
            q2 = self.target_position[i]
            if i in self.motors_with_zero_dq:
                state_zero_vector = []
                for k in range(7):
                    state_zero_vector.append([0, 0, 0, q1])
                self.state_vectors.append(state_zero_vector)
                continue

            vin = self.current_velocity[i]

            sign = abs(q2 - q1)/(q2 - q1)
            
            dv_in = self.real_max_velocity[i]-self.current_velocity[i]
            dv_out = self.real_max_velocity[i]-self.target_velocity[i]
            if dv_in != 0:
                vel_in_sign = abs(dv_in)/(dv_in)
            else:
                vel_in_sign = 1
            
            if dv_out !=0:
                vel_out_sign = abs(dv_out)/(dv_out)
            else:
                vel_out_sign = 1

            state_vector = []

            for k in range(7):
                if k in [1, 3, 5]:
                    j = 0
                elif k == 0:
                    j = vel_in_sign*sign*jmax
                elif k == 2:
                    j = -vel_in_sign*sign*jmax
                elif k == 4:
                    j = -vel_out_sign*sign*jmax
                else:
                    j = vel_out_sign*sign*jmax

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

    def modify_motor_times_parameter(self, index, base_index=None, t_1_3_=None, t_5_7_=None, t4_=None):

        if index in self.motors_with_zero_dq:
            base_index_ = self.get_index_base_motor()
            self.time_stamp[index] = self.time_stamp[base_index_]
            return

        if not base_index is None:
            t_1_3 = sum(self.time_stamp[base_index][:3])
            t_5_7 = sum(self.time_stamp[base_index][4:])
            t4 = self.time_stamp[base_index][3]
        else:
            t_1_3 = t_1_3_
            t_5_7 = t_5_7_
            t4 = t4_

        dq = self.target_position[index] - self.current_position[index]

        v = (dq - 1/2*(t_1_3*self.current_velocity[index] + t_5_7 *
                       self.target_velocity[index]))/(1/2*t_1_3+1/2*t_5_7+t4)
        
        v = round(v, 4)
        
        if v*dq < 0:
            return abs(v)*10+dq/v*100

        dv_cur = abs(v - self.current_velocity[index])
        dv_tar = abs(v - self.target_velocity[index])
        jerk = self.max_jerk[index] 
        
        t_2_2 = (t_1_3**2*jerk -
                 4*dv_cur)/jerk
        if t_2_2 > 0:
            t2 = sqrt(t_2_2)

        else:
            t2 = 0

        t1 = 1/2*t_1_3 - 1/2*t2

        t_6_2 = (t_5_7**2*jerk -
                 4*dv_tar)/jerk
        if t_6_2 > 0:
            t6 = sqrt(t_6_2)
        else:
            t6 = 0

        t5 = 1/2*t_5_7 - 1/2*t6

        new_time_stamp = [t1, t2, t1, t4, t5, t6, t5]
        self.time_stamp[index] = new_time_stamp
        return 0

    def get_max_t4(self, t13, t57):
        max_t4 = 0        
        for i in range(self.dof):
            dq = self.target_position[i] - self.current_position[i]
            t4_temp = (dq-(t13+t57)*self.real_max_velocity[i]/2-t13*self.current_velocity[i]/2-t57*self.target_velocity[i]/2)/\
                       self.real_max_velocity[i]
            if t4_temp > max_t4:
                max_t4 = t4_temp
        return max_t4

    def get_max_t13_t57(self):
        max_t13 = sum(self.time_stamp[0][:3])
        max_t57 = sum(self.time_stamp[0][4:])
        for i, el in enumerate(self.time_stamp):
            cur_t13 = sum(el[:3])
            cur_t57 = sum(el[4:])
            if cur_t13 > max_t13:
                max_t13 = cur_t13

            if cur_t57 > max_t57:
                max_t57 = cur_t57

        return [max_t13, max_t57]

    def synchronization(self):
        max_t13_t57 = self.get_max_t13_t57()
        max_t4 = self.get_max_t4(max_t13_t57[0], max_t13_t57[1])
        for i in range(self.dof):
            result = self.modify_motor_times_parameter(
                i, t_1_3_=max_t13_t57[0], t_5_7_=max_t13_t57[1], t4_=max_t4)
            if result != 0:
                raise ValueError(result)


def two_segment(segment, intermediate_waypoint):
    intermediate_velocities = []
    variable_intermediate_velocities_index = []
    upper_bounds = []
    lower_bounds = []
    bounds = []

    for i in range(segment.dof):
        if (sign(intermediate_waypoint[i] - segment.current_position[i]) == sign(segment.target_position[i]-intermediate_waypoint[i])):
            variable_intermediate_velocities_index.append(i)
            if (sign(intermediate_waypoint[i] - segment.current_position[i]) > 0):
                upper_bounds.append(segment.real_max_velocity[i])
                intermediate_velocities.append(segment.real_max_velocity[i]/2)
                lower_bounds.append(0)
                bounds.append((0, segment.real_max_velocity[i]))
            else:
                upper_bounds.append(0)
                intermediate_velocities.append-(segment.real_max_velocity[i]/2)
                lower_bounds.append(-segment.real_max_velocity[i])
                bounds.append((-segment.real_max_velocity[i], 0))

    args_ = [segment, variable_intermediate_velocities_index, intermediate_waypoint]
    if len(variable_intermediate_velocities_index) != 0:
        result = minimize(
            equationmultiple, intermediate_velocities, bounds=bounds, args=args_)
        variable_velocities = result.x
    else:
        variable_velocities = []
    intermediate_velocities = [0]*segment.dof
    for i in range(len(variable_intermediate_velocities_index)):
        intermediate_velocities[variable_intermediate_velocities_index[i]
                                ] = variable_velocities[i]

    first_segment = copy.deepcopy(segment)
    second_segment = copy.deepcopy(segment)
    first_segment.target_velocity = second_segment.current_velocity = intermediate_velocities
    first_segment.target_position = second_segment.current_position = intermediate_waypoint
    first_segment.calculate()
    second_segment.calculate()
    return [first_segment, second_segment]


def sign(a):
    if (a < 0):
        return - 1
    else:
        return 1


def equationmultiple(vars, *args):
    variable_velocity = vars
    [segment, variable_intermediate_velocities_index, intermediate_waypoint] = args[0]

    first_segment = copy.deepcopy(segment)
    second_segment = copy.deepcopy(segment)

    all_velosities = [0]*segment.dof
    for i in range(len(variable_intermediate_velocities_index)):
        all_velosities[variable_intermediate_velocities_index[i]
                       ] = variable_velocity[i]

    first_segment.target_velocity = all_velosities
    second_segment.current_velocity = first_segment.target_velocity
    first_segment.target_position = intermediate_waypoint
    second_segment.current_position = intermediate_waypoint
    try:
        first_segment.calculate()
        first_duration = first_segment.duration
    except ValueError as e:
        first_duration = e.args[0]
    try:
        second_segment.calculate()
        second_duration = second_segment.duration
    except ValueError as e:
        second_duration = e.args[0]
    result = first_duration + second_duration
    return result


def multi_segment(segment, intermediate_waypoints):
    segment_unit = copy.deepcopy(segment)
    segments = []
    for i, wp in enumerate(intermediate_waypoints):
        if i == (len(intermediate_waypoints) - 1):
            segment_unit.target_position = segment.target_position
            segment_prev, segment_unit = two_segment(segment_unit, wp)
            segments.append(segment_prev)
            segments.append(segment_unit)

        else:
            segment_unit.target_position = intermediate_waypoints[i+1]
            segment_prev, segment_unit = two_segment(segment_unit, wp)

            segments.append(segment_prev)
            segment_unit.erase_segment()
    return segments


segment = Segment(3)

segment.current_position = [0.0, 0.0, 0.0]
segment.target_position = [3.0, 3.0, 3.0]
segment.current_velocity = [0.0, 0.0, 0.0]
segment.target_velocity = [0.0, 0.0, 0.0]

segment.max_velocity = [1.5, 2.2, 0.8]
segment.max_acceleration = [1.8, 1.8, 1.8]
segment.max_jerk = [1.9, 1.9, 1.9]
intermediate_waypoint = [1.0, 5.0, 2.0]
intermediate_waypoints = [[2.0, 2.0, 2.0], [3.0, 3.0, 3.0], [5.0, 5.0, 3.5]]
start_time = time()
# segment.calculate()
# segment1, segment2 = two_segment(segment, intermediate_waypoint)
segments = multi_segment(segment, intermediate_waypoints)
print(time()-start_time)
# plot([segment])
plot(segments)
# plot([segment1, segment2])
