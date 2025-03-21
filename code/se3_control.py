import numpy as np
from scipy.spatial.transform import Rotation


class SE3Control(object):
    """

    """

    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass = quad_params['mass']  # kg
        self.Ixx = quad_params['Ixx']  # kg*m^2
        self.Iyy = quad_params['Iyy']  # kg*m^2
        self.Izz = quad_params['Izz']  # kg*m^2
        self.arm_length = quad_params['arm_length']  # meters
        self.rotor_speed_min = quad_params['rotor_speed_min']  # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max']  # rad/s
        self.k_thrust = quad_params['k_thrust']  # N/(rad/s)**2
        self.k_drag = quad_params['k_drag']  # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz]))  # kg*m^2
        self.g = 9.81  # m/s^2

        # STUDENT CODE HERE
        # For Gradescope
        self.Kp = np.diag([6., 6., 40])  # 4
        self.Kd = np.diag([5, 5, 10])  # 6
        self.Kr = np.diag([200, 200, 40])
        self.Kw = np.diag([22, 22, 10])

    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        R = (Rotation.from_quat(state['q'])).as_matrix()
        # Calculate F_des using Eq 31, 32, 33
        r_ddot_des = (flat_output['x_ddot'].reshape(3,1)
                      - self.k_d @ (state['v'].reshape(3,1) - flat_output['x_dot'].reshape(3,1))
                      - self.k_p @ (state['x'].reshape(3,1) - flat_output['x'].reshape(3,1)))

        F_des = self.mass * r_ddot_des.reshape([3, 1]) + np.array([0, 0, self.mass* self.g]).reshape([3, 1])

        # Compute u_1 from Eq 34
        b_3 = R[:, 2].reshape([3, 1])
        u_1 = b_3.T @ F_des

        # Determine R_des from Eq 38 and the definition of b_i,des
        b_3_des = F_des / np.linalg.norm(F_des)
        psi_T = flat_output['yaw']
        a_psi = np.array([np.cos(psi_T), np.sin(psi_T), 0]).reshape([3, 1])
        b3Xa = np.cross(b_3_des, a_psi, axis=0)
        b_2_des = b3Xa / np.linalg.norm(b3Xa)
        R_des = np.hstack([np.cross(b_2_des, b_3_des, axis=0), b_2_des, b_3_des])

        # Find the error orientation e_R from Eq 39 and substitute w for e_w
        e_R = 0.5 * (R_des.T @ R - R.T @ R_des)
        e_R3 = np.array([e_R[2, 1], e_R[0, 2], e_R[1, 0]])
        w_des = np.zeros_like(state['w'])
        e_w = state['w'] - w_des

        # Compute u_2 from Eq 40
        u_2 = (self.inertia @ (-self.k_r @ e_R3 - self.k_w @ e_w)).reshape([3, 1])

        gamma = self.k_drag / self.k_thrust
        l = self.arm_length
        A = np.array([
            [1, 1, 1, 1],
            [0, l, 0, -l],
            [-l, 0, l, 0],
            [gamma, -gamma, gamma, -gamma]])

        u = np.vstack((u_1.reshape(1, 1), u_2.reshape((-1, 1))))
        F = np.linalg.inv(A) @ u
        F[F < 0] = 0
        w = np.clip(np.sqrt(F / self.k_thrust), self.rotor_speed_min, self.rotor_speed_max)


        cmd_motor_speeds = w.reshape([4])
        cmd_thrust = F.sum()
        cmd_moment = u[1:].reshape([3])
        cmd_q = state['q']

        control_input = {'cmd_motor_speeds': cmd_motor_speeds,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment,
                         'cmd_q': cmd_q}
        return control_input