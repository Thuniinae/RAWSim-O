import pandas as pd
import utils

# Read the CSV file
files, folder = utils.askFiles("footprints.csv")
for file in files:
    print(f"fixing: {file}")
    df = pd.read_csv(file, sep=';')

    # Check if the columns 'NBots' and 'Instance' exist in the dataframe
    if 'NBots' in df.columns and 'Instance' in df.columns:
        # Add the 'NBots' column values to the 'Instance' column values
        df['Instance'] = df.apply(lambda row: f"{row['Instance']}r{row['NBots']}", axis=1)

        # Write the modified dataframe back to the CSV file
        df.to_csv(file, index=False, sep=';')
    else:
        print("Columns 'NBots' and/or 'Instance' not found in the CSV file.")
