import pandas as pd

def normalize(): 

  for gait in ["walk", "trot", "gallop"]:
    for dist in ["05", "1", "2"]:
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
    
normalize()