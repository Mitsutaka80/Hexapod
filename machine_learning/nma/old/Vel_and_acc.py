from modules.camera_convert import createXYangT
from modules.servo_convert import create_servo_dict
import argparse
import matplotlib.pyplot as plt

def differentiate(fx):
    fxd = {}
    fxd['t'] = []
    for key in fx.keys():
        fxd[key] = []
            #fx[key] = differentiate(robot_data[key],robot_data['t'])
        for i in range(len(fx['t']))[1:]:
            if key!='t':
                fxd[key].append((fx[key][i]-fx[key][i-1])/(fx['t'][i]-fx['t'][i-1]))
            else:
                fxd['t'].append((fx['t'][i]+fx['t'][i-1])/2)
    return fxd

parser = argparse.ArgumentParser()
parser.add_argument('-c','--camera',help='Camera data file to convert')
parser.add_argument('-s','--servo',help='Servo data file from Arduino')
args = parser.parse_args()

robot_data = createXYangT(args.camera)
servo_data = create_servo_dict(args.servo)
robot_vel = differentiate(robot_data)
servo_vel = differentiate(servo_data)
robot_acc = differentiate(robot_vel)
servo_acc = differentiate(servo_vel)

#print robot_acc 
#print servo_acc 
ax1 = plt.subplot()
ax2 = ax1.twinx()
ax1.plot(robot_data['t'],robot_data['x'])
#ax2.plot(servo_acc['t'],servo_acc['3'],'r-')
ax2.plot(servo_data['t'],servo_data['3'],'r-')
#ax2.plot(servo_data['t'],servo_data['3'],'r-')
plt.show()
def find_percentile(alist,percent):
    val = sorted(alist)[int(percent*len(alist))]
    return val
 find_percentile(robot_acc['x'],0.75)
