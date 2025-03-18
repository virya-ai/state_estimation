#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np 
import rospy


def plot(gt_df, encoder_df ):

    # Convert all required columns to NumPy arrays
    gt_x = gt_df['Gt_x'].to_numpy()
    gt_y = gt_df['Gt_y'].to_numpy()

    pred_x = encoder_df['x'].to_numpy()
    pred_y = encoder_df['y'].to_numpy()


    # Plot Ground Truth and EKF trajectories
    plt.figure(figsize=(10, 6))
    plt.plot(gt_x, gt_y, label='Ground Truth', color='g', marker='x', linestyle='-', markersize=5)
    plt.plot(pred_x, pred_y, label='Encoder Odometry', color='r', marker='.', linestyle='-', markersize=5)

    # Add labels and title
    plt.xlabel('X values')
    plt.ylabel('Y values')
    plt.title('Plot of X vs Y')
    plt.grid(True)
    plt.legend()
    # plt.xlim((-6, 7))
    # plt.ylim((-3, 10))
    plt.show()

    

def plot_csv_data(run_no):


    folder_path = "/home/catkin_ws/src/data_processing/data/online_plotting"
    file_path = folder_path + f"/run_no{run_no}/bag2csv.csv"
    encoder_file_path = folder_path + f"/run_no{run_no}/offline_encoder_odom.csv"
    
    df = pd.read_csv(file_path)
    encoder_df = pd.read_csv(encoder_file_path)
    plot(df, encoder_df)


if __name__ == "__main__":

    plot_csv_data(279)
