import pandas as pd

def normalize(): 

  for spine in ["s", "ns"]:
    for num in ["1", "2", "3"]:
             
      file_path = "balance_" + spine + "_" + num + ".csv"
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

def format_balance_data():
  df_s1 = pd.read_csv("balance_s_1.csv")
  df_s2 = pd.read_csv("balance_s_2.csv")
  df_s3 = pd.read_csv("balance_s_3.csv")

  df_ns1 = pd.read_csv("balance_ns_1.csv")
  df_ns2 = pd.read_csv("balance_ns_2.csv")
  df_ns3 = pd.read_csv("balance_ns_3.csv")

  # remove these columns:

  for df in [df_s1, df_s2, df_s3, df_ns1, df_ns2, df_ns3]:
    df.drop(columns=["t", "x", "y"], inplace=True)
    
  # calculate the average of the three tests for each spine
  average_s = df_s1.add(df_s2).add(df_s3).div(3)
  average_ns = df_ns1.add(df_ns2).add(df_ns3).div(3)

  # Add t column to the average dataframes
  average_s["t"] = average_s.index * 0.006
  average_ns["t"] = average_ns.index * 0.006

  # Save the average dataframes to CSV files
  average_s.to_csv("balance_s.csv", index=False)
  average_ns.to_csv("balance_ns.csv", index=False)


format_balance_data()

