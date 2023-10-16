import pandas as pd 

def save_dataframe_to_csv(df, filename='/home/orin4/Desktop/log_acc/log_acc_2023_10_10.csv'):

    try:
        old_data = pd.read_csv(filename)
        df = pd.concat([old_data, df], ignore_index=True)
        df.to_csv(filename, index=False)
        print(f"Data saved successfully to {filename}")
    except Exception as e:
        print(f"An error occurred: {e}")