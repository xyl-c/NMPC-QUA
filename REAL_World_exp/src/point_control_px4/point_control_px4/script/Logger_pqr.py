import os
import csv
import numpy as np

class Logger:

    def __init__(self, filename):
        self.filename = filename[0]
        # base_path = '/home/factslabegmc/final_wardi_files/src/quad_newton_raphson/quad_newton_raphson/data_analysis/'
        # self.full_path = os.path.join(base_path, self.filename)
        # print(f"Logging to: {self.full_path}")

        base_path = os.path.dirname(os.path.abspath(__file__))        # Get the directory where the script is located
        base_path = os.path.join(base_path, 'data_analysis/logs')
        print(f"logger {base_path = }")        # Print the base path
        self.filename = filename[0]        # Assuming 'filename' is passed or defined as a list
        self.full_path = os.path.join(base_path, self.filename)        # Combine the base path with the filename
        print(f"Logging to: {self.full_path}")      # Print the full path
        os.makedirs(os.path.dirname(self.full_path), exist_ok=True)        # Ensure the directory exists, and creates it if it doesn't


    def log(self, ControlNode):
        with open(self.full_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time',
                            'x', 'y', 'z', 'vx',
                            'vy', 'vz', 'roll', 'pitch', 'yaw', 'p', 'q', 'r',
                            'x_ref', 'y_ref', 'z_ref', 'yaw_ref', 'throttle', 'p_u', 'q_u', 'r_u', 
                            'mpc_time_history', 'ctrl_callback_time_history'
                            ])
            
            time_history = ControlNode.get_ctrl_loop_time_log()
            x_history = ControlNode.get_x_log()
            y_history = ControlNode.get_y_log()
            z_history = ControlNode.get_z_log()
            vx_history = ControlNode.get_vx_log()
            vy_history = ControlNode.get_vy_log()
            vz_history = ControlNode.get_vz_log()
            roll_history = ControlNode.get_roll_log()
            pitch_history = ControlNode.get_pitch_log()
            yaw_history = ControlNode.get_yaw_log()
            p_history = ControlNode.get_p_log()
            q_history = ControlNode.get_q_log()
            r_history = ControlNode.get_r_log()
            x_ref_history = ControlNode.get_x_ref_log()
            y_ref_history = ControlNode.get_y_ref_log() #15
            z_ref_history = ControlNode.get_z_ref_log()
            yaw_ref_history = ControlNode.get_yaw_ref_log()

            throttle_history = ControlNode.get_throttle_log()
            p_u_history = ControlNode.get_p_u_log()
            q_u_history = ControlNode.get_q_u_log()
            r_u_history = ControlNode.get_r_u_log()
            mpc_time_history = ControlNode.get_mpc_timel_log()
            ctrl_callback_time_history = ControlNode.get_ctrl_callback_timel_log() #23

            """
            def get_x_log(self): return np.array(self.x_log).reshape(-1, 1)
            def get_y_log(self): return np.array(self.y_log).reshape(-1, 1)
            def get_z_log(self): return np.array(self.z_log).reshape(-1, 1)
            def get_yaw_log(self): return np.array(self.yaw_log).reshape(-1, 1)
            def get_throttle_log(self): return np.array(self.throttle_log).reshape(-1, 1)
            def get_roll_log(self): return np.array(self.roll_log).reshape(-1, 1)
            def get_pitch_log(self): return np.array(self.pitch_log).reshape(-1, 1)
            def get_yaw_rate_log(self): return np.array(self.yaw_rate_log).reshape(-1, 1)
            def get_ref_x_log(self): return np.array(self.ref_x_log).reshape(-1, 1)
            def get_ref_y_log(self): return np.array(self.ref_y_log).reshape(-1, 1)
            def get_ref_z_log(self): return np.array(self.ref_z_log).reshape(-1, 1)
            def get_ref_yaw_log(self): return np.array(self.ref_yaw_log).reshape(-1, 1)
            def get_ctrl_loop_time_log(self): return np.array(self.ctrl_loop_time_log).reshape(-1, 1)
            def get_mpc_timel_log(self): return np.array(self.mpc_timel_array).reshape(-1, 1)
            def get_ctrl_callback_timel_log(self): return np.array(self.ctrl_callback_timel_log).reshape(-1, 1)
            def get_metadata(self): return self.metadata.reshape(-1, 1)

            """
            
            # Pad the metadata to match the time history
            # padding_length = time_history.shape[0] - metadata.shape[0]
            # metadata = np.pad(metadata, ((0, padding_length), (0, 0)), 'constant', constant_values='0')
           

            # Combine the histories for logging
            data = np.hstack((time_history, 
                              x_history, y_history, z_history, vx_history,
                              vy_history, vz_history, roll_history, pitch_history,
                              yaw_history, p_history, q_history, r_history,
                              x_ref_history, y_ref_history, z_ref_history, 
                              yaw_ref_history, throttle_history, p_u_history, q_u_history, r_u_history, 
                              mpc_time_history, ctrl_callback_time_history
                              ))
            # Write each row to the CSV file
            for row in range(data.shape[0]):
                writer.writerow(np.asarray(data[row, :]).flatten())

            print(f"\nWrote to {self.full_path}")

            


            