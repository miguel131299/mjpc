import pandas as pd

def normalize(): 

  for spine in ["Spine", "NoSpine"]:
    for gait in ["walk", "trot"]:
        for dist in ["05", "1", "2"]:
          for freq in ["1", "2", "3"]:
            for num in ["1", "2", "3"]:
             
              file_path = spine + "/" + gait + "_" + dist + "m_" + freq + "hz_" + num + ".csv"
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