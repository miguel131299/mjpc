import pandas as pd

def normalize(): 

  data = {"gait_frequencies": [1, 2, 3], "gait_frequencies_gallop": [2, 3, 4]}

  for angle in ["5", "10", "15"]:

    df = pd.DataFrame(data)

    for spine in ["Spine", "NoSpine"]:
      for gait in ["walk", "trot", "gallop"]:

        freq = []
        speeds = []
        
        if gait == "gallop":
          freq = ["2", "3", "4"]
        else:
          freq = ["1", "2", "3"]

        for f in freq:
          file_path = spine + "/" + gait + "_" + angle + "deg_" + f + "hz.csv"
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

def format_inclination_data(): 

  data = {"gait_frequencies": [1, 2, 3], "gait_frequencies_gallop": [2, 3, 4]}

  for angle in ["5", "10", "15"]:

    df = pd.DataFrame(data)

    for spine in ["Spine", "NoSpine"]:
      for gait in ["walk", "trot", "gallop"]:

        freq = []
        speeds = []
        
        if gait == "gallop":
          freq = ["2", "3", "4"]
        else:
          freq = ["1", "2", "3"]

        for f in freq:
          file_path = spine + "/" + gait + "_" + angle + "deg_" + f + "hz.csv"

          # Read the CSV file into a DataFrame
          df_single = pd.read_csv(file_path)

          # Calculate the average speed
          total_distance = 5
          total_time = df_single["t"].iloc[-1]

          speed = total_distance / total_time

          # Append the speed to the list
          speeds.append(speed)

        # Create a new column in the DataFrame for the y-axis data
        df["{gait}_{spine}".format(gait=gait, spine=spine)] = speeds

    # Save the DataFrame to a CSV file
    df.to_csv("inclination_" + angle + "deg.csv", index=False)

format_inclination_data()

          


    