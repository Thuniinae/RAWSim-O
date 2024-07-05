import subprocess
from subprocess import Popen, DEVNULL
from datetime import datetime, timedelta, timezone
import random
import time


working_directory = "D:\codes\RAWSim-O\RAWSimO.CLI"
experimentFolder = "D:\codes\RAWSim-OData\debug"
InstanceFolder = "D:\codes\RAWSim-O\Material\Instances\SquareLayouts"
#InstanceFolder = r"D:\codes\RAWSim-O\Material\Instances\VarsRatioLayouts"
SettingFolder = "D:\codes\RAWSim-O\Material\configFiles"
controllerFolder = "D:\codes\RAWSim-O\Material\configFiles"
Iteration = 2
ParalleledNum = 1

# Add all combinations
FileNames = [("Sv1r60", "JOSi100o500q", "SAIn")]
stop = False


# run experiment
cmds = []
for i in range(Iteration):
    for instance, setting, controller in FileNames:
        instPath = "{}\{}.xlayo".format(InstanceFolder, instance)
        settPath = "{}\{}.xsett".format(SettingFolder, setting)
        contPath = "{}\{}.xconf".format(controllerFolder, controller)
        outputDir = experimentFolder
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
   proc = Popen(cmds[NextURLNo], cwd=working_directory)
   print ("Started to Process: ", FileNames[NextURLNo % (len(FileNames))])
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
      time.sleep(1) # You may wish to change the time for this
      CheckRunning()

   print ("Done!")