import pandas as pd

def normalize(): 

  for spine in ["Spine", "NoSpine"]:
    for gait in ["walk", "trot", "gallop"]:
      for angle in ["5", "10", "15"]:
        freq = []
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
    
normalize()