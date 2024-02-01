import pandas as pd

gait_names = ["walk", "trot", "gallop"]

def format_straight_speed_data(): 
  # create x-axis columns (gait frequency)
  data = {"gait_frequencies": [1, 2, 3], "gait_frequencies_gallop": [2, 3, 4]}

  # create dataframe with both x-axis columns
  df = pd.DataFrame(data)

  for gait in gait_names:
      # read the data from the files

      for spine in ["s", "ns"]:
          
        frequencies = []
        speeds = []
        
        if gait == "gallop":
            frequencies = [2, 3, 4]
        else:
            frequencies = [1, 2, 3]
            
        for frequency in frequencies:

          total_time = 0

          for test_number in range(1, 4):
          
            # Load the CSV file into a DataFrame
            file_path = "/home/miguel/Documents/mujoco_mpc/Thesis Data/Straight/{gait}_{frequency}hz_{spine}_{test_number}.csv".format(gait=gait, frequency=frequency, spine=spine, test_number=test_number)

            df_single = pd.read_csv(file_path)

            total_time += df_single["t"].iloc[-1]

          # Calculate the average speed
          total_distance = 15
          speed = total_distance / total_time

          # Append the speed to the list
          speeds.append(speed)

          # Create a new column in the DataFrame for the y-axis data
        df["{gait}_{spine}".format(gait=gait, spine=spine)] = speeds

  # Save the DataFrame to a CSV file
  df.to_csv("speed_data.csv", index=False)

def format_straight_angle_data():
   
  df = pd.DataFrame()
   
  for gait in gait_names:
    for spine in ["s", "ns"]:
      file_path = "/home/miguel/Documents/mujoco_mpc/Thesis Data/Straight/data/angles/{gait}_{spine}.csv".format(gait=gait, spine=spine)

      df_single = pd.read_csv(file_path)

    
      # Normalize the values in 't' between 0 and 2
      min_value = df_single["t"].min()
      max_value = df_single["t"].max()

      df_single["t"] = (df_single["t"] - min_value) / (max_value - min_value) * 2

      # Append the data to the DataFrame

      df["{gait}_{spine}_t".format(gait=gait, spine=spine)] = df_single["t"]

      for angle in ["yaw", "pitch", "roll"]:
        df["{gait}_{spine}_{angle}".format(gait=gait, spine=spine, angle=angle)] = df_single[angle]

  ## Save average (mean) and standard deviation of the angles in CSV
        
  data_df = pd.DataFrame({
    "gait": [],
    "spine": [],
    "angle": [],
    "mean": [],
    "std": [],
    "min": [],
    "max": []
  })
  
  for gait in gait_names:
    for spine in ["s", "ns"]:
      for angle in ["yaw", "pitch", "roll"]:
        data_df = data_df._append({
          "gait": gait,
          "spine": spine,
          "angle": angle,
          "mean": df["{gait}_{spine}_{angle}".format(gait=gait, spine=spine, angle=angle)].mean(),
          "std": df["{gait}_{spine}_{angle}".format(gait=gait, spine=spine, angle=angle)].std(),
          "min": df["{gait}_{spine}_{angle}".format(gait=gait, spine=spine, angle=angle)].min(),
          "max": df["{gait}_{spine}_{angle}".format(gait=gait, spine=spine, angle=angle)].max()
        }, ignore_index=True)
  

  ## Save all the data in CSV
  
  data_df.to_csv("angle_stats.csv", index=False)
      

  # df.to_csv("angle_data.csv", index=False)

format_straight_angle_data()
   