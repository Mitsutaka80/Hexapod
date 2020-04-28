from modules.camera_convert import createXYangT
from modules.servo_convert import create_servo_dict
import argparse
import sys
import matplotlib.pyplot as plt
import numpy as np
import math

def differentiate(fx):
    fxd = {}
    fxd['t'] = []
    for key in fx.keys():
        fxd[key] = []
        for i in range(len(fx['t']))[1:-1]:
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
        for key in dict2ex.keys():
            if key!='t':
                y2 = dict2ex[key][high]
                y1 = dict2ex[key][low]
                m = (y2-y1)/(dict2ex['t'][high]-dict2ex['t'][low])
                c = y1 - m*dict2ex['t'][low]
                new_dict[key].append(m*dict1['t'][i]+c)
    new_dict['t'] = dict1['t']
    return new_dict

def double_diff(f,xi,xj,t):
    x0i = xi[t-1]
    x1i = xi[t]
    x2i = xi[t+1]
    x0j = xj[t-1]
    x1j = xj[t]
    x2j = xj[t+1]
    f0 = f[t-1]
    f1 = f[t]
    f2 = f[t+1]
    if x0i==x1i:
        x0i-=0.0001
    if x1i==x2i:
        x2i-=0.0001
    if x0j==x2j:
        x0j-=0.0001
    dp5 = (f1-f0)/(x1i-x0i)
    d1p5= (f2-f1)/(x2i-x1i)
    dx = (x2j-x0j)/2
    val = (d1p5-dp5)/dx
    return val 
    
def calculate_H(t,rob,ser):
    nums = len(ser)-1
    H = np.zeros((nums,nums))
    for i in range(len(H)):
        for j in range(len(H[i])):
            H[i][j] = double_diff(rob['x'],ser[str(i)],ser[str(j)],t)
    return H

parser = argparse.ArgumentParser()
parser.add_argument('-c','--camera',help='Camera data file to convert')
parser.add_argument('-s','--servo',help='Servo data file from Arduino')
args = parser.parse_args()

robot_data = createXYangT(args.camera)
servo_data = create_servo_dict(args.servo)
#print servo_data.keys()
#servo_new = extrapolate(robot_data,servo_data)
#print servo_data
summed = []
for i in range(len(servo_data['t'])):
    tot = 0
    for key in servo_data.keys():
        if key!='t':
            tot+=(servo_data[key][i]-1500)
    summed.append(tot)
#print len(servo_data['t']), len(summed)
#plt.plot(servo_data['t'],summed)
#plt.show()
robot_new = extrapolate(servo_data,robot_data)
min_dy = 100000
min_rotat = 100000
loc = 0
n = 1
forwards = {}
gra = []
for t in range(len(robot_new['t']))[n:-n]:
    val = double_diff(robot_new['x'],robot_new['y'],robot_new['ang'],t)
    gra.append(val)
    if val>0:
        if str(loc) not in forwards:
            forwards[str(loc)] = [t]
        else:
            forwards[str(loc)].append(t)
    else:
        loc+=1

ylim = 100
xlim = 4000
plt.plot(robot_new['t'][n:-n],gra)
#plt.ylim([-ylim,ylim])
#plt.xlim([0,xlim])
#plt.show()

#print forwards
max_key = max(forwards,key=lambda x: len(set(forwards[x])))
#print max_key
to_h = forwards[max_key][1:-1]
numServos = len(servo_data)-1

servo_traj = {}
for ser in range(numServos):
    servo_traj[str(ser)] = {'shift':[],'period':[]}
for to in to_h:
    H = calculate_H(to,robot_new,servo_data)
    H = np.asmatrix(H)
    w,v = np.linalg.eig(H)
    eigen_loc = np.argmax(np.absolute(w))
    eigenval = w[eigen_loc].real
    eigenvec = v[:,eigen_loc].real
    #print v[:,eigen_loc]
    for key in range(numServos):
        try:
            x0 = 0
            x1 = 1
            TIMEstep = servo_data['t'][eigen_loc+x1]-servo_data['t'][eigen_loc]
            #print eigenvec[key]
            #TIMEstep = 100
            y0 = servo_data[str(key)][eigen_loc]
            y1 = round(eigenvec[key]*TIMEstep+y0,0)
            #print y0,y1
            #print servo_data['t'][eigen_loc+x1]-servo_data['t'][eigen_loc]
            timestep = 1
            lhs0 = math.asin((y0-1500)/140)
            lhs1 = math.asin((y1-1500)/140)
            #print y0,y1
            period = (2*math.pi*timestep*(x1-x0))/(lhs1-lhs0)
            shift = lhs1-(2*math.pi*x1)/period
            period = int(round(period))
            #servo_traj[str(key)] = {'shift':shift,'period':period}
            servo_traj[str(key)]['shift'].append(shift)
            servo_traj[str(key)]['period'].append(period)
        except:
            pass

#print servo_traj
for key,val in servo_traj.iteritems():
    if len(val)!=0:
        for KEY,VAL in val.iteritems():
            if len(VAL)!=0:
                val[KEY] = sum(VAL)/len(VAL)
#print servo_traj
for key, val in servo_traj.iteritems():
    if val['period'] == []:
        servo_traj[key]['period'] = 0
        servo_traj[key]['shift'] = 0
        

f = open('period_shift.h','w')
f.write('#ifndef period_shift_h\n#define period_shift_h\nextern int periods[];\nextern float shifts[];\nconst int numServos = '+str(numServos)+';\n#endif')
f.close()
f = open('period_shift.cpp','w')
f.write('#include "period_shift.h"\nint periods[numServos] = {')

for sh in range(len(servo_traj.keys()))[:-1]:
    f.write(str(servo_traj[str(sh)]['period'])+', ')
f.write(str(servo_traj[str(numServos-1)]['period']))
f.write('};\nfloat shifts[numServos] = {')
for sh in range(len(servo_traj.keys()))[:-1]:
    f.write(str(servo_traj[str(sh)]['shift'])+', ')
f.write(str(servo_traj[str(numServos-1)]['shift']))
f.write('};')
f.close()
