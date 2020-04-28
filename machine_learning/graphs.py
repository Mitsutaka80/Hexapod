from modules.camera_convert import createXYangT
from modules.servo_convert import create_servo_dict
import argparse
import matplotlib.pyplot as plt
import numpy as np
import sys
import scipy as sci
import scipy.fftpack as fftpack
import scipy.signal as signal
import glob

def differentiate(fx):
    fxd = {}
    for key in fx.keys():
        fxd[key] = [0.0]
        for i in range(len(fx['t']))[1:]:
            if key!='t':
                fxd[key].append((fx[key][i]-fx[key][i-1])/(fx['t'][i]-fx['t'][i-1]))
            else:
                fxd['t'].append(fx['t'][i])
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

def accelerate(f,xi,xj):
    val = []
    for t in range(len(f))[1:-2]:
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
        val.append((d1p5-dp5)/dx)
    return val 
    
def calculate_H(t,rob,ser):
    nums = len(ser)-1
    H = np.zeros((nums,nums))
    for i in range(len(H)):
        for j in range(len(H[i])):
            H[i][j] = double_diff(rob['x'],ser[str(i)],ser[str(j)],t)
    return H

parser = argparse.ArgumentParser()
#parser.add_argument('-c','--camera',help='Camera data file to convert')
#parser.add_argument('-s','--servo',help='Servo data file from Arduino')
parser.add_argument('-w','--which',help='First second third etc...')
args = parser.parse_args()

start = 1500
amp = 140
THE_FORWARDS = {}
names = ['fft','nma','peaks']
fig = plt.figure(figsize=(12,6))
plt.rc('axes',labelsize=16)
for nam in names:
    rob_str = nam+'_missing/robot_data/*'+args.which+'*'
    ser_str = nam+'_missing/servo_data/*'+args.which+'*'
    rob_str = glob.glob(rob_str)[0]
    ser_str = glob.glob(ser_str)[0]
    #print rob_str, ser_str
    robot_data = createXYangT(rob_str)
    servo_data = create_servo_dict(ser_str)
    numServos = len(servo_data.keys())-1
    #servo_new = extrapolate(robot_data,servo_data)
    #servo_new = extrapolate(robot_data,servo_data)
    #print robot_data
    #robot_new = extrapolate(servo_data,robot_data)
    robot_diff = {'forwards':[0],'sideways':[0]}
    for t in range(len(robot_data['t']))[1:]:
        x0 = robot_data['x'][t-1]
        x1 = robot_data['x'][t]
        y0 = robot_data['y'][t-1]
        y1 = robot_data['y'][t]
        phi = np.arctan2(y1-y0,x1-x0)-robot_data['ang'][t-1]
        hyp = np.sqrt((y1-y0)**2+(x1-x0)**2)
        robot_diff['forwards'].append(hyp*np.cos(phi)+robot_diff['forwards'][t-1])
        robot_diff['sideways'].append(hyp*np.sin(phi)+robot_diff['sideways'][t-1])
    
    robot_diff['t'] = robot_data['t']
    robot_diff['angle'] = robot_data['ang']
    robot_new = extrapolate(servo_data,robot_diff)
    robot_vel = differentiate(robot_new)
    loc = 0
    forwards = {}
    gra = []
    for t in range(len(robot_vel['t'])):
        if robot_vel['forwards'][t]>0:
            gra.append(t)
            if str(loc) not in forwards:
                forwards[str(loc)] = [t]
            else:
                forwards[str(loc)].append(t)
            loc -= 1
        loc += 1
    
    lein = 8
    for key,val in forwards.iteritems():
        for k,v in forwards.iteritems():
            if 0<v[0]-val[-1]<lein*len(val):
                fill = [item+val[-1] for item in range(v[0]-val[-1])[1:]]
                forwards[key]=val+fill+v
                #print v[0]-val[-1]
            elif 0<val[0]-v[-1]<lein*len(val):
                fill = [item+v[-1] for item in range(val[0]-v[-1])[1:]]
                forwards[key]=v+fill+val
                #print val[0]-v[-1]
    
    summed = []
    for i in range(len(servo_data['t'])):
        tot = 0
        for key in servo_data.keys():
            if key!='t':
                tot+=(servo_data[key][i]-start)
        summed.append(tot)
    max_key = max(forwards,key=lambda x: len(set(forwards[x])))
    times = robot_new['t'][forwards[max_key][0]:forwards[max_key][-1]+1]
    to_fft= robot_new['forwards'][forwards[max_key][0]:forwards[max_key][-1]+1]
    THE_FORWARDS[nam]=to_fft
    #plt.plot(robot_diff['t'],robot_diff['forwards'],label='$Method = %s$' % (nam))
    plt.plot(robot_new['t'],robot_new['forwards'],label='$Method = %s$' % (nam))
    to_fft = [val+40 for val in to_fft]
    vis = [val+20 for val in robot_new['forwards']]

#plt.plot(robot_diff['t'], robot_diff['forwards'],label='$Original$')
#plt.plot(robot_new['t'], vis,label='$Extrapolated$')
#plt.plot(times,to_fft,label='$Selected\, data$')
plt.legend(loc='upper left')
plt.ylabel('$Distance\, travelled\, forwards\, (Pixels)$')
plt.xlabel('$Time\, (ms)$')
plt.show()
fig.savefig('/Users/cwiseman/Documents/uni_work/FYP_limping_bot/pics4report/forwards_5legs'+args.which+'.png')
