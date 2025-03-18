#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np 
import rospy
import re

def merge_dfs(df1, df2):
    iter1 = 0
    iter2 = 0
    rows = []

    # Loop through both dataframes until we've processed all rows
    while (iter1 < len(df1) or iter2 < len(df2)):
        if (iter1 >= len(df1)):  # All rows from df1 have been processed, take remaining rows from df2
            timestamp, x, y = df2.iloc[iter2, :]
            rows.append([timestamp, np.nan, np.nan, x, y])  # Add NaN for df1 values
            iter2 += 1
        elif (iter2 >= len(df2)):  # All rows from df2 have been processed, take remaining rows from df1
            timestamp, x, y = df1.iloc[iter1, :]
            rows.append([timestamp, x, y, np.nan, np.nan])  # Add NaN for df2 values
            iter1 += 1
        else:
            t1 = df1.iloc[iter1, 0]  # Timestamp from df1
            t2 = df2.iloc[iter2, 0]  # Timestamp from df2

            if (t1 < t2):  # Timestamp from df1 is earlier
                timestamp, x, y = df1.iloc[iter1, :]
                rows.append([timestamp, x, y, np.nan, np.nan])
                iter1 += 1
            elif (t1 > t2):  # Timestamp from df2 is earlier
                timestamp, x, y = df2.iloc[iter2, :]
                rows.append([timestamp, np.nan, np.nan, x, y])
                iter2 += 1
            else:  # Both timestamps are the same, take values from both
                timestamp, x1, y1 = df1.iloc[iter1, :]
                _, x2, y2 = df2.iloc[iter2, :]
                rows.append([timestamp, x1, y1, x2, y2])  # Combine values from both
                iter1 += 1
                iter2 += 1

    # Convert rows to a DataFrame
    out_df = pd.DataFrame(rows, columns=['timestamp', 'x_gt', 'y_gt', 'x_ekf', 'y_ekf'])
    out_df = out_df.interpolate(method='linear', axis=0)

    return out_df

def fit_polynomial(x, y, degree=10):
    # Fit a polynomial of specified degree to the data points
    poly_x = np.polyfit(range(len(x)), x, degree)
    poly_y = np.polyfit(range(len(y)), y, degree)

    # Evaluate the polynomial for smoothed values
    smooth_x = np.polyval(poly_x, range(len(x)))
    smooth_y = np.polyval(poly_y, range(len(y)))

    return smooth_x, smooth_y

def plot_old(gt_df, ekf_df):
    ekf_df = ekf_df.loc[ekf_df['x'] != 0]
    gt_df = gt_df.loc[gt_df['x'] > 0.0041]
    ekf_df= ekf_df.reset_index(drop=True)
    gt_df = gt_df.reset_index(drop=True)
    

    gt_df['timestamp'] -= gt_df['timestamp'][0]
    ekf_df['timestamp'] -= ekf_df['timestamp'][0]


    x_gt = gt_df['x'].to_numpy()  - gt_df.iloc[0, 1]
    y_gt = gt_df['y'].to_numpy() - gt_df.iloc[0, 2]
    x_ekf = ekf_df['x'].to_numpy() 
    y_ekf = ekf_df['y'].to_numpy() 

    # x_, y_ = x_ekf[470], y_ekf[470]
    # theta = - 0 * np.pi/180
    # for i in range(len(x_ekf)):
    #     r = np.sqrt(x_ekf[i]**2 + y_ekf[i]**2)
    #     t = np.arctan2(y_ekf[i], x_ekf[i])
    #     x_ekf[i] = r * np.cos(theta + t) 
    #     y_ekf[i] = r* np.sin(theta + t) 

    # Plot the data
    plt.figure(figsize=(10, 6))  # Set the figure size (optional)
    # plt.plot(x_values, y_values, label='Simple Kinematics (Encoder)', color='b', marker='o', linestyle='-', markersize=5)
    # plt.plot(x_des, y_des, label='Desired Trajectory', color='y', marker='x', linestyle='-', markersize=5)
    plt.plot(x_gt, y_gt, label='Ground Truth', color='r', marker='x', linestyle='-', markersize=5)
    plt.plot(x_ekf, y_ekf, label='Kalman filter (Encoder + imu)', color='g', marker='.', linestyle='-', markersize=5)
    
    # Add labels and title
    plt.xlabel('X values')  # X-axis label
    plt.ylabel('Y values')  # Y-axis label
    plt.title('Plot of X vs Y')  # Title of the plot
    plt.grid(True)  # Show grid
    plt.legend()  # Show legend
    # plt.ylim([-5, 5])
    
    # Display the plot
    # plt.show()

    
    

    print(ekf_df.head())

    df1 = gt_df.copy()
    df2 = ekf_df.copy()
    # df1.set_index('timestamp', inplace=True)
    # df2.set_index('timestamp', inplace=True)
    # error_df = df1.join(df2, how='outer')
    print(df1.head())
    print(df2.head())
    # error_df = pd.merge(df1, df2, on = 'timestamp', how='outer')
    error_df = merge_dfs(df1, df2)
    print(error_df.head(200))
    errors_x = error_df['x_ekf'].to_numpy() - error_df['x_gt'].to_numpy()
    errors_y = error_df['y_ekf'].to_numpy() - error_df['y_gt'].to_numpy()

    plt.figure(figsize=(12, 6))

    # Plot  X
    plt.subplot(2, 2, 1)  # (rows, columns, subplot index)
    plt.plot(gt_df['timestamp'].to_numpy(), gt_df['x'].to_numpy(), label="X (GT)", color="green")
    plt.plot(ekf_df['timestamp'].to_numpy(), ekf_df['x'].to_numpy(), label="X (EKF)", color="red")
    plt.xlabel('Time')
    plt.ylabel('X Position')
    plt.title('Ground Truth vs EKF X Position')
    plt.legend()
    plt.grid()

    # Plot Y
    plt.subplot(2, 2, 2)  # (rows, columns, subplot index)
    plt.plot(gt_df['timestamp'].to_numpy(), gt_df['y'].to_numpy(), label="Y (GT)", color="green")
    plt.plot(ekf_df['timestamp'].to_numpy(), ekf_df['y'].to_numpy(), label="Y (EKF)", color="red")
    plt.xlabel('Time')
    plt.ylabel('Y Position')
    plt.title('Ground Truth vs EKF Y Position')
    plt.legend()
    plt.grid()

    # # Plot error in X
    plt.subplot(2, 2, 3)  # (rows, columns, subplot index)
    plt.plot(error_df['timestamp'].to_numpy(), errors_x, label="Drift (m)", color="red")
    plt.xlabel('Time')
    plt.ylabel('Drift (m)')
    plt.title('X axis Drift')
    plt.legend()
    plt.grid()

    # # Plot error in Y
    plt.subplot(2, 2, 4)  # (rows, columns, subplot index)
    plt.plot(error_df['timestamp'].to_numpy(), errors_y, label="Drift (m)", color="red")
    plt.xlabel('Time')
    plt.ylabel('Drift (m)')
    plt.title('Y axis Drift')
    plt.legend()
    plt.grid()

    plt.tight_layout()  # Adjust subplots for better layout
    plt.show()


def plot_new(df, folder_path):   

    # Convert all required columns to NumPy arrays
    timestamp = df['timestamp'].to_numpy()
    gt_x = df['gt_x'].to_numpy()
    gt_y = df['gt_y'].to_numpy()
    ekf_x = df['ekf_x'].to_numpy()
    ekf_y = df['ekf_y'].to_numpy()
    gt_theta = df['gt_theta'].to_numpy() * 180 / np.pi
    ekf_theta = df['ekf_theta'].to_numpy() * 180 / np.pi
    drift_x = df['drift_x'].to_numpy()
    drift_y = df['drift_y'].to_numpy()
    drift_abs = df['drift_abs'].to_numpy()
    drift_theta = df['drift_theta'].to_numpy() * 180 / np.pi
    ekf_vel_x = df['ekf_vel_x'].to_numpy()
    ekf_vel_y = df['ekf_vel_y'].to_numpy()
    gt_vel_x = df['gt_vel_x'].to_numpy()
    gt_vel_y = df['gt_vel_y'].to_numpy()

    # Plot Ground Truth and EKF trajectories
    plt.figure(figsize=(10, 6))
    plt.plot(gt_x, gt_y, label='Ground Truth', color='g', marker='x', linestyle='-', markersize=5)
    plt.plot(ekf_x, ekf_y, label='Kalman Filter (EKF)', color='r', marker='.', linestyle='-', markersize=5)

    # Add labels and title
    plt.xlabel('X values')
    plt.ylabel('Y values')
    plt.title('Plot of X vs Y')
    plt.grid(True)
    plt.legend()
    # plt.ylim((-3, 3))
    plt.axis('equal')
    plt.savefig(folder_path + "/plot_XvsY.png")
    # plt.show()

    # Time series plots for X and Y positions and drift
    plt.figure(figsize=(12, 6))

    # Plot X positions over time
    plt.subplot(2, 2, 1)
    plt.plot(timestamp, gt_x, label="X (GT)", color="green")
    plt.plot(timestamp, ekf_x, label="X (EKF)", color="red")
    plt.xlabel('Time')
    plt.ylabel('X Position')
    plt.title('Ground Truth vs EKF X Position')
    plt.legend()
    plt.grid()

    # Plot Y positions over time
    plt.subplot(2, 2, 2)
    plt.plot(timestamp, gt_y, label="Y (GT)", color="green")
    plt.plot(timestamp, ekf_y, label="Y (EKF)", color="red")
    plt.xlabel('Time')
    plt.ylabel('Y Position')
    plt.title('Ground Truth vs EKF Y Position')
    plt.legend()
    plt.grid()

    # Plot X drift over time
    plt.subplot(2, 2, 3)
    plt.plot(timestamp, drift_x, label="Drift X (m)", color="red")
    plt.xlabel('Time')
    plt.ylabel('Drift X')
    plt.title('X Drift Over Time')
    plt.legend()
    plt.grid()

    # Plot Y drift over time
    plt.subplot(2, 2, 4)
    plt.plot(timestamp, drift_y, label="Drift Y (m)", color="red")
    plt.xlabel('Time')
    plt.ylabel('Drift Y')
    plt.title('Y Drift Over Time')
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.savefig(folder_path + "/Drift1.png")
    # plt.show()

    # Plot velocities, their drift, and thetas with their drift
    plt.figure(figsize=(15, 10))

    # Plot X velocities
    plt.subplot(2, 2, 1)
    plt.plot(timestamp, gt_vel_x, label="GT Velocity X", color="green")
    plt.plot(timestamp, ekf_vel_x, label="EKF Velocity X", color="red")
    plt.xlabel('Time')
    plt.ylabel('Velocity X')
    plt.title('Ground Truth vs EKF Velocity (X)')
    plt.legend()
    plt.grid()

    # Plot Y velocities
    plt.subplot(2, 2, 2)
    plt.plot(timestamp, gt_vel_y, label="GT Velocity Y", color="green")
    plt.plot(timestamp, ekf_vel_y, label="EKF Velocity Y", color="red")
    plt.xlabel('Time')
    plt.ylabel('Velocity Y')
    plt.title('Ground Truth vs EKF Velocity (Y)')
    plt.legend()
    plt.grid()

    # Plot drift in velocities (X)
    plt.subplot(2, 2, 3)
    velocity_drift_x = -(ekf_vel_x - gt_vel_x)
    plt.plot(timestamp, velocity_drift_x, label="Velocity Drift X", color="red")
    plt.xlabel('Time')
    plt.ylabel('Velocity Drift')
    plt.title('Velocity Drift (X)')
    plt.legend()
    plt.grid()

    # Plot drift in velocities (Y)
    plt.subplot(2, 2, 4)
    velocity_drift_y = -(ekf_vel_y - gt_vel_y)
    plt.plot(timestamp, velocity_drift_y, label="Velocity Drift Y", color="red")
    plt.xlabel('Time')
    plt.ylabel('Velocity Drift')
    plt.title('Velocity Drift (Y)')
    plt.legend()
    plt.grid()
    plt.savefig(folder_path + "/Drift2.png")

    plt.figure(figsize=(15, 10))
    # Plot thetas (angles)
    plt.subplot(2, 2, 1)
    plt.plot(timestamp, gt_theta, label="GT Theta", color="green")
    plt.plot(timestamp, ekf_theta, label="EKF Theta", color="red")
    plt.xlabel('Time')
    plt.ylabel('Theta (degrees)')
    plt.title('Robot Orientation (GT vs EKF)')
    plt.legend()
    plt.grid()

    plt.subplot(2, 2, 2)
    plt.plot(timestamp, drift_x, label="Drift X (m)", color="blue")
    plt.plot(timestamp, drift_y, label="Drift Y (m)", color="red")
    plt.plot(timestamp, drift_abs, label="Drift abs (m)", color="black")
    plt.xlabel('Time')
    plt.ylabel('Drift (m)')
    plt.title('Drift Over Time')
    plt.legend()
    plt.grid()

    # Plot drift in thetas
    plt.subplot(2, 2, 3)
    plt.plot(timestamp, drift_theta, label="Theta Drift", color="red")
    plt.xlabel('Time')
    plt.ylabel('Theta Drift (degrees)')
    plt.title('Theta Drift Over Time')
    plt.legend()
    plt.grid()

    plt.tight_layout(pad = 3)
    plt.savefig(folder_path + "/Drift3.png")
    plt.show()


def plot_csv_data(run_no, gt = "ndt"):


    folder_path = "/home/catkin_ws/src/data_processing/data/online_plotting"
    gt_path = ""
    if gt =="fast_lio":
        gt_path = folder_path + f"/run_no{run_no}/fast_lio_odom.csv"
    else:
        gt_path = folder_path + f"/run_no{run_no}/ndt_odom.csv"

    ekf_path = folder_path + f"/run_no{run_no}/ekf_odom.csv"
    kinematic_path = folder_path + f"/run_no{run_no}/encoder_odom.csv"
    drift_data_path = folder_path + f"/run_no{run_no}/drift_log.csv"

    gt_df = pd.read_csv(gt_path)
    ekf_df = pd.read_csv(ekf_path)

    df = pd.read_csv(drift_data_path)

    folder_path += f"/run_no{run_no}"
    plot_new(df, folder_path)

if __name__ == "__main__":

    plot_csv_data(435, gt="ndt")
