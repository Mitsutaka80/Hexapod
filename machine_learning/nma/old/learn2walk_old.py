from modules.camera_convert import createXYangT
from modules.servo_convert import create_servo_dict
import argparse
import matplotlib.pyplot as plt
import numpy as np

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

def extrapolate(dict1,dict2ex):
    new_dict = {}
    for key in dict2ex.keys():
        if key!='t':
            new_dict[key]=[dict2ex[key][0]]
    for i in range(len(dict1['t']))[1:]:
        compare = range(len(dict2ex['t']))
        #print compare
        #print range(len(dict2ex['t']))
        for j in range(len(dict2ex['t'])):
            compare[j] = abs(dict1['t'][i]-dict2ex['t'][j])
        closest = compare.index(min(compare))
        compare[closest] *= 10000000
        close = compare.index(min(compare))
        high=low=0
        if closest>close:
            high = closest
            low = close
        else:
            high = close
            low = closest
        #time_dict[str(i)] = {'low':low,'high':high}
        for key in dict2ex.keys():
            if key!='t':
                y2 = dict2ex[key][high]
                y1 = dict2ex[key][low]
                m = (y2-y1)/(dict2ex['t'][high]-dict2ex['t'][low])
                c = y1 - m*dict2ex['t'][low]
                new_dict[key].append(m*dict1['t'][i]+c)
    new_dict['t'] = dict1['t']
    return new_dict

parser = argparse.ArgumentParser()
parser.add_argument('-c','--camera',help='Camera data file to convert')
parser.add_argument('-s','--servo',help='Servo data file from Arduino')
args = parser.parse_args()

robot_data = createXYangT(args.camera)
servo_data = create_servo_dict(args.servo)
#servo_new = extrapolate(robot_data,servo_data)
robot_new = extrapolate(servo_data,robot_data)
#print servo_new
numServos = len(servo_data.keys())-1
#print numServos
eigen_dict = {}
for t in range(len(robot_new['t']))[1:-1]:
    H = np.zeros((numServos,numServos))
    for i in range(len(H)):
        for j in range(len(H[i])):
            try: 
                d1 = (robot_data['x'][t]-robot_data['x'][t-1])/(servo_data[str(i)][t]-servo_data[str(i)][t-1])
                d2 = (robot_data['x'][t+1]-robot_data['x'][t])/(servo_data[str(i)][t+1]-servo_data[str(i)][t])
                dx = (servo_data[str(j)][t+1]-servo_data[str(j)][t-1])/2
                H[i][j] = (d2-d1)/dx
            except:
                H[i][j] = 10000000000
    if 10000000000 not in H:
        H = np.asmatrix(H)
        eigenvals,eigenvects = np.linalg.eig(H)
        for eig in range(len(eigenvals)):
            if not np.iscomplex(eigenvals[eig]) and not np.iscomplex(eigenvects[eig]).any():
                #if not np.iscomplex(eigenvects[eig]).any():
                eigen_dict[str(eigenvals[eig].real)] = eigenvects[eig].real
        #print eigenvals,eigenvects
        #eigenval_list.append(eigenvals)
#eigenval_list = np.transpose(np.asarray(eigenval_list))
#print eigenval_list
print eigen_dict
#mean_eigen = []
#for ei in eigenval_list:
#    mean_eigen.append(sum(ei)/len(ei))
#    plt.plot(range(len(ei)),ei)
#print mean_eigen
##plt.ylim(-0.1,0.1)
#plt.show()

###################
#test = []
#for i in range(len(servo_data['t'])):
#    val = 0
#    for key in servo_data.keys():
#        if key!='t':
#            val+=servo_data[key][i]
#    test.append(val)
##robot_vel = differentiate(robot_data)
##servo_vel = differentiate(servo_data)
##robot_acc = differentiate(robot_vel)
##servo_acc = differentiate(servo_vel)
##print robot_acc 
##print servo_acc 
#ax1 = plt.subplot()
#ax2 = ax1.twinx()
##ax1.plot(robot_data['t'],robot_data['x'],'b-')
#ax1.plot(robot_new['t'],robot_new['x'],'b-')
##ax2.plot(servo_new['t'],servo_new['3'],'g-')
##ax2.plot(servo_data['t'],test,'r-')
#ax2.plot(servo_data['t'],servo_data['3'],'r-')
#plt.show()
#def find_percentile(alist,percent):
#    val = sorted(alist)[int(percent*len(alist))]
#    return val
##find_percentile(robot_acc['x'],0.75)
