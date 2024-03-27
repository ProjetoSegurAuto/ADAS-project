import pandas as pd 
from datetime import datetime
import errno

today = datetime.today().strftime('%Y-%m-%d')
FILENAME = '/home/orin1/Desktop/log_acc/log_acc_'+today+'.csv'

def save_dataframe_to_csv(linha_arquivo, columns, filename=FILENAME):
    try:
        df = pd.DataFrame(linha_arquivo, columns=columns)
        old_data = pd.read_csv(filename)
        df = pd.concat([old_data, df], ignore_index=True)
        df.to_csv(filename, index=False)
        
    except Exception as e:
        # 'No such file or directory'
        df.to_csv(filename, index=False)

    finally:
        print(f"Data saved successfully to {filename}")

