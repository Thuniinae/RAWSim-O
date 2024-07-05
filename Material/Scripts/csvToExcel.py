import pandas as pd
import os
from subprocess import Popen, PIPE
from openpyxl import load_workbook
from time import sleep
import utils

averageTemplatePath = r"D:\codes\RAWSim-OData\compare\averagesTemplate.xlsx"
summaryTemplatePath = r"D:\codes\RAWSim-OData\compare\summaryTemplate.xlsx"

files, folder = utils.askFiles("footprints.csv")
summary = []
summaryPath = os.path.join(folder, f"summary_{os.path.basename(folder)}.xlsx")
for filepath in files:
    print(f"processing: {filepath}")
    directory, filename = os.path.split(filepath)
    #filepath = input('Enter the Folder: ').strip('"')
    inputPath = os.path.abspath(filepath)
    outputPath = os.path.join(directory, f"footprints_{os.path.basename(directory)}.xlsx")
    averagePath = os.path.join(directory, f"averages_{os.path.basename(directory)}.xlsx")
    
    # read footprints.csv
    df = pd.read_csv(inputPath, delimiter=';')
    df.to_excel(outputPath)

    # calculate average of combination of instance and controller
    columns = ["Instance", "Setting", "Controller","ItemThroughputRate", "DistanceTraveled", 
               "TimingDecisionsOverall", "TimingPathPlanningOverall", "TimingTaskAllocationOverall", "TimingItemStorageOverall", 
                "TimingPodStorageOverall", "TimingRepositioningOverall", "TimingReplenishmentBatchingOverall", 
                "TimingOrderBatchingOverall", "TimingPodSelectionOverall", 
                 "OSIdleTimeAvg", "ItemPileOneAvg", "OrderPileOneAvg", "OrderLatenessAvg", "LateOrdersFractional", "DistanceTraveledPerBot"]
    result = df.loc[:, columns]
    avg = result.groupby(["Instance", "Setting", "Controller"]).agg({key: ['mean', 'std'] for key in columns[3:]}).reset_index() 
    # copy excel template
    Popen(["copy", averageTemplatePath, averagePath], shell=True)
    sleep(1)
    writer = pd.ExcelWriter(averagePath, engine = 'openpyxl', mode='a', if_sheet_exists = "replace")
    avg.to_excel(writer, sheet_name="Data")
    writer.close()

    # average result from all robots
    avgs = avg.drop(columns = [col for col in avg.columns if col[1] == 'std'])
    avgs.columns = [col[0] if isinstance(col, tuple) else col for col in avgs.columns] # flatten
    avgs['Instance'] = avgs['Instance'].str.extract(r'(.+?)r\d+$')
    avgs = avgs.groupby(["Instance", "Setting", "Controller"]).mean().reset_index() 
    # put average in averages
    summary.append(avgs)

print("creating summary")
# Concatenate all average to summary
summary_df = pd.concat(summary, ignore_index=True)
# copy excel template
Popen(["copy", summaryTemplatePath, summaryPath], shell=True)
sleep(1)
writer = pd.ExcelWriter(summaryPath, engine = 'openpyxl', mode='a', if_sheet_exists = "replace")
summary_df.to_excel(writer, sheet_name="Data")
writer.close()
print('Done')



