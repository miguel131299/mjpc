import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def XYPlot():

    # Replace 'your_file.csv' with the actual CSV file path.
    csv_file = '/home/miguel/Documents/mujoco_mpc/build/bin/data.csv'
    # Read the CSV file into a Pandas DataFrame.
    df = pd.read_csv(csv_file)

    # Extract columns from the DataFrame.
    x_position = df['x']
    y_position = df['y']

    # Define custom limits for the X and Y axes.
    x_min = -5  # Replace with your desired minimum X value.
    x_max = 5   # Replace with your desired maximum X value.
    y_min = -5  # Replace with your desired minimum Y value.
    y_max = 5   # Replace with your desired maximum Y value.


    # Create a scatter plot.
    fig, ax = plt.subplots(figsize=(10, 6))  # Adjust figure size if needed.
    sc = ax.scatter([], [], marker='o', label='Data Points')

    # Set the custom limits for the X and Y axes.
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)


    # Set axis labels and title.
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Animated XY Plane Plot')

    # Function to update the scatter plot for each frame.
    def update(frame):
        sc.set_offsets([[x_position[frame], y_position[frame]]])
        return sc,

    # Create the animation.
    ani = FuncAnimation(fig, update, frames=len(x_position), blit=True, interval=100)

    # To save the animation as a video (optional).
    # ani.save('animation.mp4', writer='ffmpeg')

    # Show the animation (you may need to adjust the interval and repeat options).
    plt.grid(True)
    plt.show()

def AnglesPlot():
    # Replace 'your_file.csv' with the actual CSV file path.
    csv_file = '/home/miguel/Documents/mujoco_mpc/build/bin/data.csv'

    # Read the CSV file into a Pandas DataFrame.
    df = pd.read_csv(csv_file)

    # Extract columns from the DataFrame.
    time = df['t']
    yaw = df['yaw']
    pitch = df['pitch']
    roll = df['roll']

    plt.plot(time, yaw, label='Yaw')
    plt.plot(time, pitch, label='Pitch')
    plt.plot(time, roll, label='Roll')

    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.title('Euler Angles')

    plt.legend()

    plt.grid(True)

    plt.show()

def normalizeTime():

    file_names = [
        "walk_1hz_s_", "walk_2hz_s_", "walk_3hz_s_","trot_1hz_s_", "trot_2hz_s_", "trot_3hz_s_","gallop_2hz_s_", "gallop_3hz_s_", "gallop_4hz_s_",
                  "walk_1hz_ns_", "walk_2hz_ns_", "walk_3hz_ns_","trot_1hz_ns_", "trot_2hz_ns_", "trot_3hz_ns_","gallop_2hz_ns_", "gallop_3hz_ns_", "gallop_4hz_ns_",
                  ]
    
    for file_name in file_names:

        for i in range(1, 4):

            # Load the CSV file into a DataFrame
            file_path = '/home/miguel/Documents/Thesis Data/Straight/' + file_name + str(i) + '.csv'
            print(file_path)
            df = pd.read_csv(file_path)

            # Get the value of the first time row
            first_time_value = df['t'][0]

            # Subtract the first_time_value from all rows in the "t" column
            df['t'] = round(df['t'] - first_time_value, 5)

            # Save the modified DataFrame back to the same CSV file
            df.to_csv(file_path, index=False)



normalizeTime()