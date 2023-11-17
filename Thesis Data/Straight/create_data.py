import pandas as pd
import csv

# Define the CSV file path
csv_file_path = "speed_data.csv"

gait_names = ["walk", "trot", "gallop"]

# Open the CSV file in write mode
with open(csv_file_path, mode='w', newline='') as file:
    # Create a CSV writer object
    writer = csv.writer(file)
    
    # Write the header row
    writer.writerow(["name", "gait frequency", "speed"])

    for gait in gait_names:
        # read the data from the files

        for spine in ["s", "ns"]:
            frequencies = []
            if gait == "gallop":
                frequencies = [2, 3, 4]
            else:
                frequencies = [1, 2, 3]

            for frequency in frequencies:

              total_time = 0

              for test_number in range(1, 4):
              
                # Load the CSV file into a DataFrame
                file_path = "/home/miguel/Documents/Thesis Data/Straight/{gait}_{frequency}hz_{spine}_{test_number}.csv".format(gait=gait, frequency=frequency, spine=spine, test_number=test_number)

                print(file_path)

                df = pd.read_csv(file_path)

                total_time += df["t"].iloc[-1]

              # Calculate the average speed
              total_distance = 15
              speed = total_distance / total_time

              writer.writerow(["{gait}_{spine}".format(gait=gait, spine=spine), frequency, speed])








            



print("Done!")