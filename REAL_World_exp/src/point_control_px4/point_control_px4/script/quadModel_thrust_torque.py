import numpy as np
import casadi as ca

from matplotlib import pyplot as plt
from acados_template import AcadosModel

from casadi import vertcat, horzcat, diag



class QuadrotorModel(object):
    def __init__(self, Ts, N):
        self.model = AcadosModel()
        self.model.name = 'quadrotor_ned'

        self.Ts = Ts
        self.N = N

        # =========================
        # Physical parameters
        # =========================
        self.g = 9.79136
        self.m = 1.287
        # self.I = np.diag([
        #     0.02166666666666667,
        #     0.02166666666666667,
        #     0.04000000000000001
        # ])
        self.I = np.diag([
            0.00416,
            0.004855,
            0.005544
        ])

        # state: [px, py, pz, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
        self.nx = 13
        # input: [T, Mx, My, Mz]
        self.nu = 4

        # =========================
        # Input constraints
        # =========================
        self.f_max = 40.0
        self.f_min = 0.0

        self.mx_max = 0.5
        self.mx_min = -0.5

        self.my_max = 0.5
        self.my_min = -0.5

        self.mz_max = 0.216
        self.mz_min = -0.216

        # =========================
        # State constraints
        # =========================
        self.vx_max = 5.0
        self.vx_min = -5.0

        self.vy_max = 5.0
        self.vy_min = -5.0

        self.vz_max = 5.0
        self.vz_min = -5.0

        self.wx_max = 1.0
        self.wx_min = -1.0

        self.wy_max = 1.0
        self.wy_min = -1.0

        self.wz_max = 1.0
        self.wz_min = -1.0

        # =========================
        # Cost weights
        # 这里仍沿用你的原设置
        # =========================
        # self.Q = np.diag([200., 200., 500., 200.])
        # self.R = diag(horzcat(1., 700.0 * 1., 700.0 * 1., 700.0 * 1.))
        # self.P_ = np.diag([200., 200., 200., 200.])

        self.Q = np.diag([200., 200., 500., 200.])
        self.R = diag(horzcat(1., 300.0 * 1., 300.0 * 1., 300.0 * 1.))
        self.P_ = np.diag([400., 400., 500., 400.])

        # self.Q = np.diag([100., 100., 300., 100.])
        # self.R = diag(horzcat(1., 300.0 * 1., 300.0 * 1., 300.0 * 1.))
        # self.P_ = np.diag([400., 400., 500., 400.])

    def SetupOde(self):
        # =========================
        # States
        # NED inertial frame:
        # x -> North, y -> East, z -> Down
        # =========================
        px_s = ca.SX.sym('px')
        py_s = ca.SX.sym('py')
        pz_s = ca.SX.sym('pz')

        qw_s = ca.SX.sym('qw')
        qx_s = ca.SX.sym('qx')
        qy_s = ca.SX.sym('qy')
        qz_s = ca.SX.sym('qz')

        vx_s = ca.SX.sym('vx')
        vy_s = ca.SX.sym('vy')
        vz_s = ca.SX.sym('vz')

        wx_s = ca.SX.sym('wx')
        wy_s = ca.SX.sym('wy')
        wz_s = ca.SX.sym('wz')

        x = ca.vertcat(
            px_s, py_s, pz_s,
            qw_s, qx_s, qy_s, qz_s,
            vx_s, vy_s, vz_s,
            wx_s, wy_s, wz_s
        )

        # =========================
        # Inputs
        # u = [T, Mx, My, Mz]
        # =========================
        u = ca.SX.sym('u', 4)

        T = u[0]
        Mx = u[1]
        My = u[2]
        Mz = u[3]

        # =========================
        # Quaternion to rotation matrix
        # Assume quaternion represents body -> world (NED) rotation
        # =========================
        qw = x[3]
        qx = x[4]
        qy = x[5]
        qz = x[6]

        R_bw = ca.vertcat(
            ca.horzcat(1 - 2*qy**2 - 2*qz**2,
                       2*qx*qy - 2*qz*qw,
                       2*qx*qz + 2*qy*qw),

            ca.horzcat(2*qx*qy + 2*qz*qw,
                       1 - 2*qx**2 - 2*qz**2,
                       2*qy*qz - 2*qx*qw),

            ca.horzcat(2*qx*qz - 2*qy*qw,
                       2*qy*qz + 2*qx*qw,
                       1 - 2*qx**2 - 2*qy**2)
        )

        # =========================
        # Quaternion kinematics
        # q_dot = 0.5 * Omega(omega) * q
        # =========================
        wx = x[10]
        wy = x[11]
        wz = x[12]

        Omega = ca.vertcat(
            ca.horzcat(0,   -wx, -wy, -wz),
            ca.horzcat(wx,   0,   wz, -wy),
            ca.horzcat(wy,  -wz,  0,   wx),
            ca.horzcat(wz,   wy, -wx,  0)
        )

        q = x[3:7]
        q_dot = 0.5 * Omega @ q

        # =========================
        # Translational dynamics in NED
        #
        # NED:
        #   z-axis points downward
        #   gravity = [0, 0, +mg]
        #
        # To keep positive T meaning "lift magnitude",
        # thrust acts upward in world, so in body frame use [0, 0, -T]
        # and transform it to world(NED):
        #   F_thrust = R_bw @ [0, 0, -T]
        # =========================
        Fg = ca.vertcat(0, 0, self.m * self.g)
        F_thrust = R_bw @ ca.vertcat(0, 0, -T)

        acc = (Fg + F_thrust) / self.m

        # =========================
        # Rotational dynamics
        # I * omega_dot + omega x (I*omega) = M
        # =========================
        omega = x[10:13]
        I_ca = ca.DM(self.I)
        I_inv_ca = ca.DM(np.linalg.inv(self.I))

        M = ca.vertcat(Mx, My, Mz)
        omega_dot = I_inv_ca @ (M - ca.cross(omega, I_ca @ omega))

        # =========================
        # Continuous-time dynamics
        # =========================
        xdot = ca.vertcat(
            x[7:10],   # p_dot = v
            q_dot,     # q_dot
            acc,       # v_dot
            omega_dot  # w_dot
        )

        # =========================
        # Acados model assignment
        # =========================
        xdot_sym = ca.SX.sym('xdot', self.nx)

        self.model.x = x
        self.model.u = u
        self.model.xdot = xdot_sym
        self.model.f_expl_expr = xdot
        self.model.f_impl_expr = xdot_sym - xdot

        # =========================
        # RK4 discrete dynamics
        # =========================
        f_c = ca.Function('f_c', [x, u], [xdot])

        X0 = ca.SX.sym('X0', self.nx)
        U0 = ca.SX.sym('U0', self.nu)

        k1 = f_c(X0, U0)
        k2 = f_c(X0 + (self.Ts / 2.0) * k1, U0)
        k3 = f_c(X0 + (self.Ts / 2.0) * k2, U0)
        k4 = f_c(X0 + self.Ts * k3, U0)

        x_next = X0 + (self.Ts / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        # quaternion normalization
        q_next = x_next[3:7]
        q_norm = ca.sqrt(ca.sumsqr(q_next) + 1e-12)
        q_next_normalized = q_next / q_norm

        x_next = ca.vertcat(
            x_next[0:3],
            q_next_normalized,
            x_next[7:13]
        )

        self.f = ca.Function('f', [X0, U0], [x_next])

        return self.model


    def step_update(self,x_k,u_k):
        s_kp1 = self.f(x_k, u_k[:, 0])
        return s_kp1







