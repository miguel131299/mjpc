import pandas as pd

def normalize(): 

  for radius in ["05", "1", "2"]:

    

    for spine in ["Spine", "NoSpine"]:
      for freq in ["1", "2", "3"]:
        for gait in ["walk", "trot"]:
          for num in ["1", "2", "3"]:
             
              file_path = spine + "/" + gait + "_" + radius + "m_" + freq + "hz_" + num + ".csv"
              print(file_path)

              # Read the CSV file into a DataFrame
              df = pd.read_csv(file_path)

              # # Select the column you want to subtract by its first element
              column_name = 't'
              selected_column = df[column_name]

              # # Subtract each element of the column by the first element
              df[column_name] = selected_column - selected_column.iloc[0]

              # # Write the modified DataFrame back to a CSV file
              df.to_csv(file_path, index=False)
    

def format_curve_speed_data():
  # create different dataframes for each radiusance
  df_05 = pd.DataFrame()
  df_1 = pd.DataFrame()
  df_2 = pd.DataFrame()

  # Drop index column


  # Add the x-axis columns to each dataframe
  df_05["gait_frequencies"] = [1, 2, 3]
  df_1["gait_frequencies"] = [1, 2, 3]
  df_2["gait_frequencies"] = [1, 2, 3]

  # iterate through the different combinations of radius, spine, gait and frequency


  for radius in ["05", "1", "2"]:
    for spine in ["Spine", "NoSpine"]:
      for gait in ["walk", "trot"]:
          
        speeds = []

        for freq in ["1", "2", "3"]:

          # create a list to store the speeds for each test
          total_time = 0


          for num in ["1", "2", "3"]:

            file_path = spine + "/" + gait + "_" + radius + "m_" + freq + "hz_" + num + ".csv"
            df = pd.read_csv(file_path)

            # get the last time value
            total_time += df["t"].iloc[-1]

          # Calculate the distance (quarter of a circle * radius)
          quarter_circunference = 0.25 * 3.1415
          if radius == "05":
            distance = quarter_circunference * 0.5

          elif radius == "1":
            distance = quarter_circunference * 1

          elif radius == "2":
            distance = quarter_circunference * 2

          # Calculate the average speed
            
          avg_speed = distance / total_time

          # append the average speed to the speeds list
          speeds.append(avg_speed)

        # append the average speed to the dataframe
        if radius == "05":
          print(speeds)
          df_05["{gait}_{spine}".format(gait=gait, spine=spine)] = speeds
        elif radius == "1":
          df_1["{gait}_{spine}".format(gait=gait, spine=spine)] = speeds
        elif radius == "2":
          df_2["{gait}_{spine}".format(gait=gait, spine=spine)] = speeds

  # Save the DataFrame to a CSV file
  print(df_05)
  df_05.to_csv("curve_data_05.csv", index=False)
  df_1.to_csv("curve_data_1.csv", index=False)
  df_2.to_csv("curve_data_2.csv", index=False)

format_curve_speed_data()