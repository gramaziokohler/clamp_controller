import compas_rrc as rrc
from compas.geometry import Frame, Point, Vector
from compas_fab.backends.ros import RosClient
import time
from datetime import datetime



ros = RosClient("192.168.0.120")
ros.run()

robot_11 = rrc.AbbClient(ros, '/rob1')

print('Connected.')
time.sleep(1.5)


# Set used robot
robot = robot_11
# The mass (weight) of the load in kg.
# Must be bigger then 0
# mass = 57.1
mass = 37
# The center of gravity of the payload expressed in mm in the tool coordinate system.
# Minimum 1 value bigger then 0
cog_x = -770
cog_y = 0
cog_z = 240
# The orientation of the axes of moment.
# These are the principal axes of the payload moment of inertia with origin in center of gravity.
# Expressed in quaternians
aom_q1 = 1
aom_q2 = 0
aom_q3 = 0
aom_q4 = 0
# The moment of inertia of the load around the axis of moment expressed in kgm2.
# Correct definition of the moments of inertia will allow optimal utilization of the path planner and axes control.
# This may be of special importance when handling large sheets of metal, and so on.
# All moments of inertia ix, iy, and iz equal to 0 kgm2 imply a point mass.
# Normally, the moments of inertia must only be defined when the distance from the mounting flange to the center of gravity
# is less than the maximal dimension of the load.
inertia_x = 0
inertia_y = 155
inertia_z = 155
# Grip load instruction
results = []
results.append(robot.send(rrc.CustomInstruction('r_A067_TPPlot',['Start'],[], feedback_level=1)))
results.append(robot.send(rrc.CustomInstruction('r_A067_GripLoad',[],[mass, cog_x, cog_y, cog_z, aom_q1, aom_q2, aom_q3, aom_q4, inertia_x, inertia_y, inertia_z], feedback_level=1)))
results.append(robot.send(rrc.CustomInstruction('r_A067_TPPlot',['End'],[], feedback_level=1)))

while not all([r.done for r in results]):
    print ([r.done for r in results])
    time.sleep(1)


# Time necessary to send all remaining commands.
time.sleep(1)
ros.terminate()
time.sleep(1)
print('ros terminated')

# Grip unload instruction
# done = robot.send_and_wait(rrc.CustomInstruction('r_A067_GripUnload',[],[]))