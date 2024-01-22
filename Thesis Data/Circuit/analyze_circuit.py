import pandas as pd

def normalize(): 

  for gait in ["walk", "trot", "gallop"]:
      for spine in ["s", "ns"]:
        frequencies = []

        if spine == "s" and gait == "gallop":
          frequencies = ["25", "3", "4"]
        elif spine == "ns" and gait == "gallop":
          frequencies = ["2", "25", "3", "4"]
        else:
          frequencies = ["1", "2", "3"]

        for freq in frequencies:
          
          file_path = "circuit_" + gait + "_" + freq + "hz_" + spine + ".csv"
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
    
def format_circuit_data():

  data = {"gait_frequencies": [1, 2, 3], "gait_frequencies_gallop": [2.5, 3, 4]}

  df = pd.DataFrame(data)
    
  for gait in ["walk", "trot", "gallop"]:
    for spine in ["s", "ns"]:
      frequencies = []

      if gait == "gallop":
        frequencies = ["25", "3", "4"]
      else:
        frequencies = ["1", "2", "3"]

      speeds = []

      for freq in frequencies:

        # Load the CSV file into a DataFrame
        file_path = "circuit_" + gait + "_" + freq + "hz_" + spine + ".csv"

        df_single = pd.read_csv(file_path)

        # create a list to store the speeds for each test
        total_time = df_single["t"].iloc[-1]
        # Calculate the average speed
        total_distance = 28 + 2 * 3.1415
        speed = total_distance / total_time

        # Append the speed to the list
        speeds.append(speed)

      # Create a new column in the DataFrame for the y-axis data
      df["{gait}_{spine}".format(gait=gait, spine=spine)] = speeds

  # Save the DataFrame to a CSV file
  df.to_csv("circuit_speed_data.csv", index=False)

format_circuit_data()