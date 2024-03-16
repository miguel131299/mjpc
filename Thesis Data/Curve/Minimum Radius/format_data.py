
import pandas as pd

def format_minimum_curve_data():
  
  df = pd.DataFrame()

  # Add the x-axis columns to each dataframe
  df["gait_frequencies"] = [1, 2, 3]

  # iterate through the different combinations of spine, gait, frequency and trial number
  for spine in ["s", "ns"]:
    for gait in ["walk", "trot"]:

      lengths = []

      for freq in ["1", "2", "3"]:

        y_sum = 0

        for num in ["1", "2", "3"]:
                    # create the file path
          file_path = gait + "_" + spine + "_" + freq + "hz_" + num + ".csv"

          # Read the CSV file into a DataFrame
          df_single = pd.read_csv(file_path)

          # get absolute value of last y value and add it to the sum
          y_sum += abs(df_single["y"].iloc[-1])

        # get the average y value
        avg_y = y_sum / 3
        
        # substract half the length of the robot
        avg_y -= 0.2

        # append the average y value to the lengths list
        lengths.append(avg_y)

      # append the average y value to the dataframe
      df["{gait}_{spine}".format(gait=gait, spine=spine)] = lengths


  # Save the DataFrame to a CSV file
  print(df)
  df.to_csv("minimum_curve_data.csv", index=False)


format_minimum_curve_data()