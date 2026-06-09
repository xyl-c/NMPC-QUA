import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    # Load the data from the CSV file
    data = np.genfromtxt('/home/xyl/dev_ws/build/point_control_px4/point_control_px4/script/data_analysis/logs/log.log', delimiter=',', skip_header=1)

    # Extract columns (adjust indices based on your CSV structure)
    time = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    z = -data[:, 3]
    vx = data[:, 4]
    vy = data[:, 5]
    vz = data[:, 6]
    roll = data[:, 7]*180.0/np.pi
    pitch = data[:, 8]*180.0/np.pi
    yaw = data[:, 9]*180.0/np.pi
    p = data[:, 10]
    q = data[:, 11]
    r = data[:, 12]
    x_ref = data[:, 13]
    y_ref = data[:, 14]
    z_ref = -data[:, 15]
    yaw_ref = data[:, 16]*180.0/np.pi

    throttle = data[:, 17]
    Mx = data[:, 18]
    My = data[:, 19]
    Mz = data[:, 20]

    sol_time = data[:, 21]

    plt.figure()
    plt.plot(time, sol_time, label='sol_time')
    plt.title('sol_time Position vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('sol_time (s)')
    plt.legend()


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z)
    ax.plot(x_ref, y_ref, z_ref, 'r--')
    ax.set_title('3D Trajectory')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')


    # Plotting
    plt.figure()
    plt.subplot(2, 2, 1)
    plt.plot(time, x, label='x')
    plt.plot(time, x_ref, label='x_ref', linestyle='--')
    plt.title('X Position vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('X Position (m)')
    plt.legend()
    
    plt.subplot(2, 2, 2)
    plt.plot(time, y, label='y')
    plt.plot(time, y_ref, label='y_ref', linestyle='--')
    plt.title('Y Position vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    
    plt.subplot(2, 2, 3)
    plt.plot(time, z, label='z')
    plt.plot(time, z_ref, label='z_ref', linestyle='--')
    plt.title('Z Position vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Z Position (m)')
    plt.legend()
    
    plt.subplot(2, 2, 4)
    plt.plot(time, yaw, label='yaw')
    plt.plot(time, yaw_ref, label='yaw_ref', linestyle='--')
    plt.title('Yaw vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw (rad)')
    plt.legend()



    # Plotting
    plt.figure()
    plt.subplot(2, 3, 1)
    plt.plot(time, vx, label='vx')
    plt.title('vx vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('vx (m/s)')
    plt.legend()
    
    plt.subplot(2, 3, 2)
    plt.plot(time, vy, label='vy')
    plt.title('vy vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('vy (m/s)')
    plt.legend()
    
    plt.subplot(2, 3, 3)
    plt.plot(time, vz, label='vz')
    plt.title('vz vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('vz (m/s)')
    plt.legend()
    
    plt.subplot(2, 3, 4)
    plt.plot(time, p, label='p')
    plt.title('p vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('p (rad/s)')
    plt.legend()
    
    plt.subplot(2, 3, 5)
    plt.plot(time, q, label='q')
    plt.title('q vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('q (rad/s)')
    plt.legend()
    
    plt.subplot(2, 3, 6)
    plt.plot(time, r, label='r')
    plt.title('r vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('r (rad/s)')
    plt.legend()



    # Plotting
    plt.figure()
    plt.subplot(2, 2, 1)
    plt.plot(time, throttle, label='throttle')
    plt.title('throttle vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('throttle')
    plt.legend()
    
    plt.subplot(2, 2, 2)
    plt.plot(time, Mx, label='Mx')
    plt.title('Mx vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Mx (Nm)')
    plt.legend()
    
    plt.subplot(2, 2, 3)
    plt.plot(time, My, label='My')
    plt.title('My vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('My (Nm)')
    plt.legend()
    
    plt.subplot(2, 2, 4)
    plt.plot(time, Mz, label='Mz')
    plt.title('Mz vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Mz (Nm)')
    plt.legend()

    plt.tight_layout()
    plt.show()



