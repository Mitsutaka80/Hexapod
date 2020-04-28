from modules.camera_convert import createXYangT
from modules.servo_convert import create_servo_dict
import argparse
import matplotlib.pyplot as plt
import numpy as np
import sys
from scipy.fftpack import fft
import scipy.signal as signal

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
    
def calculate_H(t,rob,ser):
    nums = len(ser)-1
    H = np.zeros((nums,nums))
    for i in range(len(H)):
        for j in range(len(H[i])):
            H[i][j] = double_diff(rob['x'],ser[str(i)],ser[str(j)],t)
    return H

servo_data = create_servo_dict('average_random.csv')
test = fft(servo_data['0'])
N = len(test)
tot = 0
for t in range(len(servo_data['t']))[1:]:
    tot+=servo_data['t'][t]-servo_data['t'][t-1]
T = tot/len(servo_data['t'])
xf = np.linspace(0.0,1.0/(2.0*T),N//2)
servoFFT = 2.0/N*np.abs(test[:N//2])
servoPer = []
for i in range(len(xf))[1:-2]:
    val = double_diff(servoFFT,xf,xf,i)
    hi = xf[i]
    if val<0 and 1000<(1/hi)<4000:
        #freqs.append(hi)
        #plt.axvline(x=hi)
        servoPer.append(round(1.0/hi))
print servoPer
serv = {}
num = 0
ts = {}
periods = []

for val in range(len(servo_data['0']))[1:]:
    x0 = servo_data['t'][val-1]
    x1 = servo_data['t'][val]
    y0 = servo_data['0'][val-1]
    y1 = servo_data['0'][val]
    if y1 == y0:
        x0 = servo_data['t'][val-4]
        y0 = servo_data['0'][val-4]
        
    lhs0 = np.arcsin((y0-1500)/140)
    lhs1 = np.arcsin((y1-1500)/140)
    #try:  
    period = int(round((2*np.pi*(x1-x0))/(lhs1-lhs0)))
    if period not in periods:
        periods.append(period)
    #except:
    #    pass
       #serv[str(num)] = [servo_data['0'][val]]
       #ts[str(num)] = [servo_data['t'][val]]
       #ts[str(num)] = [val]
   else:
        serv[str(num)].append(servo_data['0'][val])
        ts[str(num)].append(servo_data['t'][val])
       #ts[str(num)].append(val)
   num+=1
periods.sort()
#print periods

for key,val in serv.iteritems():
    bot = ts[key][val.index(min(val))]
    top = ts[key][val.index(max(val))]
    tot=[]
    for t in range(len(ts[key]))[1:]:
        tot.append(ts[key][t]-ts[key][t-1])
    tot.sort()
    T = tot[len(ts[key])//2]
    periods.append((top-bot)*2)
    #xf = np.linspace(0.0,1.0/(2.0*T),n//2)
    plt.plot(ts[key],val)
plt.plot(servo_data['t'],servo_data['0'])
plt.plot(xf,servoFFT)
print periods
plt.show()
