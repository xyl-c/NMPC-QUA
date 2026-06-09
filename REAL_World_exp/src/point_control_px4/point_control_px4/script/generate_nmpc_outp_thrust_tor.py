from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np
from point_control_px4.script.quadModel_thrust_torque import QuadrotorModel
import importlib, sys, os
from casadi import vertcat, horzcat, diag
from scipy.linalg import block_diag
import time

import casadi as ca
import scipy.linalg

class QuadrotorMPC2:
    def __init__(self, generate_c_code: bool, quadrotor: QuadrotorModel, horizon: float, num_steps: int):
        self.model = AcadosModel()
        self.quad = quadrotor
        self.model_name = 'holybro'
        self.horizon = horizon
        self.num_steps = num_steps

        self.hover_ctrl = np.array([self.quad.m * self.quad.g, 0., 0., 0.])

        self.ocp_solver = None
        self.generate_c_code = generate_c_code

        # === Put all generated artifacts under <this package>/acados_generated_files ===
        pkg_dir = os.path.dirname(os.path.abspath(__file__))   # .../nmpc_px4_pkg
        parts = pkg_dir.split(os.sep)
        parts = ["src" if part in ("build","install") else part for part in parts] # Replace 'build' with 'src' if it exists in the path
        pkg_dir = os.sep.join(parts)
        gen_root = os.path.join(pkg_dir, "acados_generated_files")
        os.makedirs(gen_root, exist_ok=True)

        # C code export dir and JSON path (both inside acados_generated_files)
        self.code_export_directory = os.path.join(gen_root, f"{self.model_name}_mpc_c_generated_code")
        self.json_path = os.path.join(gen_root, f"{self.model_name}_mpc_acados_ocp.json")

        if self.generate_c_code:
            print("\n[acados] Generating/compiling fresh MPC...\n")
            self.generate_mpc()
            print("[acados] Done.")
        else:
            try:
                print("\n[acados] Trying to load compiled MPC; will generate if not found...\n")
                sys.path.append(self.code_export_directory)
                acados_ocp_solver_pyx = importlib.import_module('acados_ocp_solver_pyx')
                self.ocp_solver = acados_ocp_solver_pyx.AcadosOcpSolverCython(
                    self.model_name, 'SQP', self.num_steps
                )
            except ImportError:
                print("[acados] Compiled MPC not found. Generating now...")
                self.generate_mpc()
                print("[acados] Done! Control stack should begin in two seconds...")
                time.sleep(2)



    def generate_mpc(self):

        # --- Define Acados OCP ---
        ocp = AcadosOcp()
        # ocp.model = self.quad.SetupOde()
        ocp.model = self.quad.SetupOde()
        ocp.code_export_directory = self.code_export_directory

        # f_expl, x, u = self.quad.dynamics()
        # # --- Define Acados Model ---
        # model = AcadosModel()
        # model.f_expl_expr = f_expl
        # model.x = x
        # model.u = u
        # model.name = self.model_name
        # # --- Define Acados OCP ---
        # ocp = AcadosOcp()
        # ocp.model = model
        # ocp.code_export_directory = self.code_export_directory


        nx = self.quad.nx
        nu = self.quad.nu
        # nx = 12
        # nu = 4
        ny = 4 + nu  # [x, y, z, psi, u]


        ocp.dims.N = self.num_steps
        ocp.solver_options.N_horizon = self.num_steps
        ocp.solver_options.tf = self.horizon

        ocp.model.p = ca.SX.sym('p', 4)
        ocp.parameter_values = np.zeros(4)

        # yaw extraction
        q0, q1, q2, q3 = ocp.model.x[3], ocp.model.x[4],ocp.model.x[5], ocp.model.x[6]
        psi = ca.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))
        psi_e = ca.atan2(ca.sin(psi - ocp.model.p[3]), ca.cos(psi - ocp.model.p[3]))

        # psi_e = ca.atan2(ca.sin(ocp.model.x[8] - ocp.model.p[3]), ca.cos(ocp.model.x[8] - ocp.model.p[3]))

        y = ca.vertcat(
            ocp.model.x[0:3] - ocp.model.p[0:3],
            psi_e,
            ocp.model.u - self.hover_ctrl
        )

        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.model.cost_y_expr = y
        ocp.cost.yref = np.zeros(ny)
        ocp.cost.W = scipy.linalg.block_diag(self.quad.Q, self.quad.R)

        ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.model.cost_y_expr_e = y[0:4]
        ocp.cost.yref_e = np.zeros(4)
        ocp.cost.W_e = self.quad.Q

        # ocp.constraints.lbx = np.array([self.quad.vx_min,self.quad.vy_min,self.quad.vz_min,
        #                                      self.quad.wx_min,self.quad.wy_min,self.quad.wz_min])
        # ocp.constraints.ubx = np.array([self.quad.vx_max,self.quad.vy_max,self.quad.vz_max,
        #                                      self.quad.wx_max,self.quad.wy_max,self.quad.wz_max])
        # ocp.constraints.idxbx = np.array([7,8,9,10,11,12])

        ocp.constraints.lbu = np.array([self.quad.f_min, self.quad.mx_min, self.quad.my_min, self.quad.mz_min])
        ocp.constraints.ubu = np.array([self.quad.f_max, self.quad.mx_max, self.quad.my_max, self.quad.mz_max])
        ocp.constraints.idxbu = np.arange(4)

        ocp.constraints.x0 = np.zeros(nx)

        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.tf = self.horizon
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1

        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # "PARTIAL_CONDENSING_HPIPM" #"FULL_CONDENSING_HPIPM" #"PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # "EXACT", "GAUSS_NEWTON"
        # ocp.solver_options.cost_discretization ="INTEGRATOR"
        ocp.solver_options.qp_solver_cond_N = int(self.num_steps / 2)
        ocp.solver_options.nlp_solver_type = "SQP_RTI"   # SQP_RTI
        ocp.solver_options.tol = 1e-4


        # --- Generate JSON into our custom folder ---
        AcadosOcpSolver.generate(ocp, json_file=self.json_path)


        # --- Compile with cython ---
        AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)

        # --- Import compiled module from our folder ---
        mod_path = os.path.join(self.code_export_directory, "acados_ocp_solver_pyx.so")
        spec = importlib.util.spec_from_file_location("acados_ocp_solver_pyx", mod_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        acados_ocp_solver_pyx = mod   # use it as usual

        # sys.path.append(self.code_export_directory)  # Add the directory to sys.path
        # acados_ocp_solver_pyx = importlib.import_module('acados_ocp_solver_pyx.o')
        print(f"nonononono\n\n")
        self.ocp_solver = acados_ocp_solver_pyx.AcadosOcpSolverCython(
            self.model_name, 'SQP', self.num_steps
        )

    def solve_mpc_control(self, x0, xd):
        N = self.num_steps
        if xd.shape[1] != N:
            raise ValueError("The reference trajectory should have the same length as the number of steps")

        # for i in range(N):
        #     y_ref = np.hstack((xd[:, i], self.hover_ctrl))
        #     self.ocp_solver.set(i, 'y_ref', y_ref)
        # self.ocp_solver.set(N, 'y_ref', xd[:, -1])
        for i in range(N):
            self.ocp_solver.set(i, "p", xd[:, i])
        self.ocp_solver.set(N, "p", xd[:, i])
        
        self.ocp_solver.set(0, 'lbx', x0)
        self.ocp_solver.set(0, 'ubx', x0)

        status = self.ocp_solver.solve()
        x_mpc = self.ocp_solver.get(0, 'x')
        u_mpc = self.ocp_solver.get(0, 'u')
        return status, x_mpc, u_mpc


