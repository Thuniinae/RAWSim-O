import subprocess
from subprocess import Popen, DEVNULL
from datetime import datetime, timedelta, timezone
import random
import time
import winsound


working_directory = "D:\codes\RAWSim-O\RAWSimO.CLI"
experimentFolder = r"D:\codes\RAWSim-OData\result0703"
InstanceFolder = "D:\codes\RAWSim-O\Material\Instances\SquareLayouts"
#InstanceFolder = r"D:\codes\RAWSim-O\Material\Instances\VarsRatioLayouts"
SettingFolder = "D:\codes\RAWSim-O\Material\configFiles"
controllerFolder = "D:\codes\RAWSim-O\Material\configFiles"
Iteration = 1
ParalleledNum = 8
# Add all combinations
FileNames = []

for instance in ["S"]:
   for version in range(5, 6, 1):
      for botNum in range(10, 110, 10):
         for setting in ["JOSi1000o500"]:
            for controller in ["HADODn", "SEQUn", "SAIn", "SEQUn", "SAIn", "SEQUn", "SAIn"]:
                  FileNames.append((f'{instance}v{version}r{botNum}', setting, controller))


'''for instance in ["PPI3v1r40"]:
    for setting in ["PPSv11q"]:
        for controller in ["PPCv5"]:
            FileNames.append((instance, setting, controller))'''



stop = False


# run experiment
cmds = []
for i in range(Iteration):
    for instance, setting, controller in FileNames:
        instPath = "{}\{}.xlayo".format(InstanceFolder, instance)
        settPath = "{}\{}.xsett".format(SettingFolder, setting)
        contPath = "{}\{}.xconf".format(controllerFolder, controller)
        outputDir = "{}\{}".format(experimentFolder, instance.rsplit('r', 1)[0])
        cmds.append(["dotnet", "run", instPath, settPath, contPath, outputDir])

NextURLNo = 0
Processes = []

def StartNew():
   """ Start a new subprocess if there is work to do """
   global NextURLNo
   global Processes

   dateTime = datetime.now().replace(tzinfo=timezone(timedelta(hours=8))).strftime("%m%d%H%M%S")
   seed = dateTime
   cmds[NextURLNo].append(seed)
   proc = Popen(cmds[NextURLNo], cwd=working_directory, stdout=DEVNULL)
   print (f"({NextURLNo+1}/{len(cmds)}) Started to Process: ", FileNames[NextURLNo % (len(FileNames))])
   NextURLNo += 1
   Processes.append(proc)

def CheckRunning():
   """ Check any running processes and start new ones if there are spare slots."""
   global Processes
   global NextURLNo

   for p in range(-len(Processes),0): # Check the processes in reverse order
      if Processes[p].poll() is not None: # If the process hasn't finished will return None
         del Processes[p] # Remove from list - this is why we needed reverse order

   while (len(Processes) < ParalleledNum and NextURLNo < len(cmds)): # More to do and some spare slots
      time.sleep(1)
      StartNew()

if __name__ == "__main__":
   CheckRunning() # This will start the max processes running
   while (len(Processes) > 0): # Some thing still going on.
      time.sleep(60) # You may wish to change the time for this
      CheckRunning()

   winsound.Beep(261, 500)
   time.sleep(0.2)
   winsound.Beep(329, 500)
   time.sleep(0.2)
   winsound.Beep(391, 500)
   time.sleep(0.2)
   winsound.Beep(523, 700)