#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os
import sys

def extract_robot_name(csv_filename):
    """
    Extracts the robot name from the CSV filename.
    Assumes the filename format is 'robotX_mcts_performance.csv'.
    
    Args:
        csv_filename (str): The filename of the CSV file.
        
    Returns:
        str: Extracted robot name (e.g., 'robot4').
    """
    base_name = os.path.basename(csv_filename)
    parts = base_name.split('_')
    if len(parts) < 2:
        print(f"Error: CSV filename '{csv_filename}' does not follow the expected format 'robotX_mcts_performance.csv'.")
        sys.exit(1)
    robot_name = parts[0]
    return robot_name

def plot_metrics(csv_path, save_dir, plots):
    """
    Reads the CSV file and plots the specified metrics.

    Args:
        csv_path (str): Path to the CSV file.
        save_dir (str): Directory where the plots will be saved.
        plots (list): List of plots to generate. Options: 'mcts', 'total_mcts', 'distance', 'collision', 'all'.
    """
    # Check if CSV file exists
    if not os.path.isfile(csv_path):
        print(f"Error: The file {csv_path} does not exist.")
        sys.exit(1)

    # Extract robot name
    robot_name = extract_robot_name(csv_path)

    # Read the CSV file
    try:
        data = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading the CSV file: {e}")
        sys.exit(1)

    # Verify required columns exist
    required_columns = ['MCTS Calls', 'Time Elapsed', 'Distance to Goal', 'collision_rate', 'Total_MCTS_calls']
    for col in required_columns:
        if col not in data.columns:
            print(f"Error: Missing required column '{col}' in the CSV file.")
            sys.exit(1)

    # Convert columns to appropriate data types
    try:
        data['MCTS Calls'] = pd.to_numeric(data['MCTS Calls'], errors='coerce')
        data['Time Elapsed'] = pd.to_numeric(data['Time Elapsed'], errors='coerce')
        data['Distance to Goal'] = pd.to_numeric(data['Distance to Goal'], errors='coerce')
        data['collision_rate'] = pd.to_numeric(data['collision_rate'], errors='coerce')
        data['Total_MCTS_calls'] = pd.to_numeric(data['Total_MCTS_calls'], errors='coerce')
    except Exception as e:
        print(f"Error converting data types: {e}")
        sys.exit(1)

    # Handle missing or NaN values
    data = data.dropna(subset=required_columns)
    
    # Create output directory if it doesn't exist
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    # Plotting
    if 'mcts' in plots or 'all' in plots:
        plt.figure(figsize=(10, 6))
        plt.plot(data['Time Elapsed'], data['MCTS Calls'], label='MCTS Calls', color='purple')
        plt.xlabel('Time Elapsed (s)')
        plt.ylabel('MCTS Calls')
        plt.title(f'{robot_name} - MCTS Calls Over Time')
        plt.legend()
        plt.grid(True)
        if save_dir:
            plot_filename = f"{robot_name}_mcts_calls_over_time.png"
            plt.savefig(os.path.join(save_dir, plot_filename))
            print(f"Plot saved as: {plot_filename}")
        plt.show()

    if 'total_mcts' in plots or 'all' in plots:
        plt.figure(figsize=(10, 6))
        plt.plot(data['Time Elapsed'], data['Total_MCTS_calls'], label='Total MCTS Calls', color='orange')
        plt.xlabel('Time Elapsed (s)')
        plt.ylabel('Total MCTS Calls')
        plt.title(f'{robot_name} - Total MCTS Calls Over Time')
        plt.legend()
        plt.grid(True)
        if save_dir:
            plot_filename = f"{robot_name}_total_mcts_calls_over_time.png"
            plt.savefig(os.path.join(save_dir, plot_filename))
            print(f"Plot saved as: {plot_filename}")
        plt.show()

    if 'distance' in plots or 'all' in plots:
        plt.figure(figsize=(10, 6))
        plt.plot(data['Time Elapsed'], data['Distance to Goal'], label='Distance to Goal', color='blue')
        plt.xlabel('Time Elapsed (s)')
        plt.ylabel('Distance to Goal')
        plt.title(f'{robot_name} - Distance to Goal Over Time')
        plt.legend()
        plt.grid(True)
        if save_dir:
            plot_filename = f"{robot_name}_distance_to_goal_over_time.png"
            plt.savefig(os.path.join(save_dir, plot_filename))
            print(f"Plot saved as: {plot_filename}")
        plt.show()

    if 'collision' in plots or 'all' in plots:
        plt.figure(figsize=(10, 6))
        plt.plot(data['Time Elapsed'], data['collision_rate'], label='Collision Rate', color='red')
        plt.xlabel('Time Elapsed (s)')
        plt.ylabel('Collision Rate')
        plt.title(f'{robot_name} - Collision Rate Over Time')
        plt.legend()
        plt.grid(True)
        if save_dir:
            plot_filename = f"{robot_name}_collision_rate_over_time.png"
            plt.savefig(os.path.join(save_dir, plot_filename))
            print(f"Plot saved as: {plot_filename}")
        plt.show()

def main():
    parser = argparse.ArgumentParser(description="Plot MCTS Metrics from CSV")
    parser.add_argument('--csv', type=str, required=True, help="Path to the CSV file (e.g., robot4_mcts_performance.csv)")
    parser.add_argument('--save_dir', type=str, default='plots', help="Directory to save the generated plots")
    parser.add_argument('--plots', type=str, nargs='+', default=['all'], help="List of plots to generate: 'mcts', 'total_mcts', 'distance', 'collision', 'all'")
    args = parser.parse_args()

    # Validate plot options
    valid_plots = {'mcts', 'total_mcts', 'distance', 'collision', 'all'}
    selected_plots = set(args.plots)
    if not selected_plots.issubset(valid_plots):
        invalid = selected_plots - valid_plots
        print(f"Error: Invalid plot options selected: {invalid}")
        print(f"Valid options are: {valid_plots}")
        sys.exit(1)

    plot_metrics(args.csv, args.save_dir, args.plots)

if __name__ == '__main__':
    main()