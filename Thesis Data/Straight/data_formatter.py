import pandas as pd

# create x-axis columns (gait frequency)
data = {"gait_frequencies": [1, 2, 3], "gait_frequencies_gallop": [2, 3, 4]}

# create dataframe with both x-axis columns
df = pd.DataFrame(data)

gait_names = ["walk", "trot", "gallop"]

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
          file_path = "/home/miguel/Documents/Thesis Data/Straight/{gait}_{frequency}hz_{spine}_{test_number}.csv".format(gait=gait, frequency=frequency, spine=spine, test_number=test_number)

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

          



