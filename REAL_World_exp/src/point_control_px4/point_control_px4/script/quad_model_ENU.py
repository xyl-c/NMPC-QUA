from casadi import SX, vertcat, cos, sin, tan


class Quadrotor:
    def __init__(self, sim: bool):
        self.sim = sim
        self.g = 9.79136
        self.m = 1.287

    def dynamics(self):
        # states in ENU frame
        px = SX.sym('px')     # x (East)
        py = SX.sym('py')     # y (North)
        pz = SX.sym('pz')     # z (Up)
        vx = SX.sym('vx')
        vy = SX.sym('vy')
        vz = SX.sym('vz')
        roll = SX.sym('roll')     # phi
        pitch = SX.sym('pitch')   # theta
        yaw = SX.sym('yaw')       # psi

        x = vertcat(px, py, pz, vx, vy, vz, roll, pitch, yaw)

        # control inputs
        thrust = SX.sym('thrust')   # total thrust, positive along body z-up opposite gravity effect
        p = SX.sym('p')             # body roll rate
        q = SX.sym('q')             # body pitch rate
        r = SX.sym('r')             # body yaw rate

        u = vertcat(thrust, p, q, r)

        # trigonometric terms
        sr = sin(roll)
        cr = cos(roll)
        sp = sin(pitch)
        cp = cos(pitch)
        sy = sin(yaw)
        cy = cos(yaw)
        tp = tan(pitch)

        # translational kinematics
        pxdot = vx
        pydot = vy
        pzdot = vz

        # translational dynamics in ENU
        vxdot = (thrust / self.m) * (cr * sp * cy + sr * sy)
        vydot = (thrust / self.m) * (cr * sp * sy - sr * cy)
        vzdot = (thrust / self.m) * (cr * cp) - self.g

        # Euler angle kinematics
        rolldot  = p + q * sr * tp + r * cr * tp
        pitchdot = q * cr - r * sr
        yawdot   = q * sr / cp + r * cr / cp

        f_expl = vertcat(
            pxdot, pydot, pzdot,
            vxdot, vydot, vzdot,
            rolldot, pitchdot, yawdot
        )

        return f_expl, x, u
    


