logfile = "TokyoCommander.2020-08-27.debug.log"
from datetime import datetime
from clamp_controller.ClampModel import ClampModel

time_format = '%Y-%m-%d %H:%M:%S,%f'


interest_time_begin = datetime.strptime('2020-08-27 12:23:46,471', time_format) # Inclusive
interest_time_end = datetime.strptime('2020-08-27 12:24:56,177', time_format)   # Inclusive

interest_process = 'app.cmd'

with open(logfile) as f:
    f = f.readlines()

extracted_log = []

for line in f:
    time, process, info, message = line.split(' - ', 3)
    time = datetime.strptime(time, time_format)
    if time >= interest_time_begin and time <= interest_time_end and process == interest_process:
        extracted_log.append((time, process, info, message))
        # print (line)

print (len(extracted_log))

# Further filtering and sorting.
clamplog1 = []
clamplog2 = []
clamplog3 = []
clamplog4 = []

clamp1 = ClampModel('1', 918, 0, 94.0, 225.0, 860.0, 1004.0)
clamp2 = ClampModel('2', 918, 0, 94.0, 225.0, 860.0, 1004.0)
clamp3 = ClampModel('3', 918, 0, 94.0, 225.0, 860.0, 1004.0)
clamp4 = ClampModel('4', 918, 0, 94.0, 225.0, 860.0, 1004.0)

def dataextract(clamp, status_string):
    clamp.update_status(status_string)
    err = int(clamp._raw_currentPosition - clamp._raw_currentTarget)
    power = clamp.currentMotorPowerPercentage
    pos = clamp.currentJawPosition
    batt = clamp.batteryPercentage
    return ([pos, err, power, batt])

for time, process, info, message in extracted_log:
    if not message.startswith('Status Update Success'): continue

    message_header, status_string = message.split('result = ', 1)
    
    if message.startswith('Status Update Success: Clamp 1'):
        clamplog1.append([(time - interest_time_begin).total_seconds()] + dataextract(clamp1, status_string))
    elif message.startswith('Status Update Success: Clamp 2'):
        clamplog2.append([(time - interest_time_begin).total_seconds()] + dataextract(clamp2, status_string))
    elif message.startswith('Status Update Success: Clamp 3'):
        clamplog3.append([(time - interest_time_begin).total_seconds()] + dataextract(clamp3, status_string))
    elif message.startswith('Status Update Success: Clamp 4'):
        clamplog4.append([(time - interest_time_begin).total_seconds()] + dataextract(clamp4, status_string))

print (len(clamplog1))
print (len(clamplog2))
print (len(clamplog3))
print (len(clamplog4))

# print (clamplog1)
clamps_of_interest = [[clamplog1, 'r', 'Clamp 1'] , [clamplog3, 'g', 'Clamp 2'], [clamplog4, 'b', 'Clamp 3']]
import matplotlib.pyplot as plt
plt.figure()

plt.subplot(131)
for clamplog, color, label_text in clamps_of_interest:
    time = [d[0] for d in clamplog]
    pos = [d[1] for d in clamplog]
    err = [d[2] for d in clamplog]
    power = [-d[3] for d in clamplog]
    batt = [d[4] for d in clamplog]
    # plt.subplot(subplot_pos)
    plt.plot(time, pos, color, label=label_text)
plt.ylabel('Clamp Position (mm)')
plt.ylim(100, 220)
plt.xlabel('Time since start (seconds)')
plt.xlim(0, 71)
plt.title('Clamp Jaw Position')
plt.legend(loc='upper right', frameon=True)

plt.subplot(132)
for clamplog, color, label_text in clamps_of_interest:
    time = [d[0] for d in clamplog]
    pos = [d[1] for d in clamplog]
    err = [d[2] for d in clamplog]
    power = [-d[3] for d in clamplog]
    batt = [d[4] for d in clamplog]
    # plt.subplot(subplot_pos)
    plt.plot(time, power, color, label=label_text)
plt.ylabel('Clamp Power (percentage of full power)')
plt.ylim(0, 45)
plt.xlabel('Time since start (seconds)')
plt.xlim(0, 71)
plt.title('Power Output')
plt.legend(loc='upper left', frameon=True)


plt.subplot(133)
for clamplog, color, label_text in clamps_of_interest:
    time = [d[0] for d in clamplog]
    pos = [d[1] for d in clamplog]
    err = [d[2]/918 for d in clamplog]
    power = [-d[3] for d in clamplog]
    batt = [d[4] for d in clamplog]
    # plt.subplot(subplot_pos)
    plt.plot(time, err, color, label=label_text)
plt.ylabel('Clamp Jaw Position Error (mm)')
plt.xlabel('Time since start (seconds)')
plt.xlim(0, 71)
plt.title('Position Error')
plt.legend(loc='upper right', frameon=True)

plt.suptitle('Plot of Clamp motion during a closure from 220mm to 100mm in 70 seconds', fontsize=14)

plt.show()
