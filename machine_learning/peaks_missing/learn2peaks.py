from modules.camera_convert import createXYangT
from modules.servo_convert import create_servo_dict
import argparse
import matplotlib.pyplot as plt
import numpy as np
import sys
import scipy as sci
import scipy.fftpack as fftpack
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

def strongest(FFT,XF,num):
    peaks = accelerate(FFT,XF,XF)
    periods = []
    tops = []
    for pea in range(len(peaks)):
        if peaks[pea]<0:
            tops.append(FFT[pea])
    tops.sort()
    tops = tops[-num:]
    for top in tops:
        val = FFT.tolist().index(top)
        if XF[val]!=0:
            periods.append(round(1/XF[val]))
    return periods

parser = argparse.ArgumentParser()
parser.add_argument('-c','--camera',help='Camera data file to convert')
parser.add_argument('-s','--servo',help='Servo data file from Arduino')
args = parser.parse_args()

start = 1500
amp = 140
robot_data = createXYangT(args.camera)
servo_data = create_servo_dict(args.servo)
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
    #robot_diff['angle'].append(robot_data['ang'][t]-robot_data['ang'][t-1])
robot_diff['t'] = robot_data['t']
robot_diff['angle'] = robot_data['ang']
robot_new = extrapolate(servo_data,robot_diff)
vis = [val+20 for val in robot_new['forwards']]

#fig = plt.figure(figsize=(12,6))
#plt.rc('axes',labelsize=16)
#plt.ylabel('$Distance\, travelled\, forwards\, (Pixels)$')
#plt.xlabel('$Time\, (ms)$')
#plt.plot(robot_diff['t'], robot_diff['forwards'],label='$Original$')
#plt.plot(robot_new['t'], vis,label='$Extrapolated$')
#plt.legend(loc="lower right")
#plt.show()
##fig.savefig('/Users/cwiseman/Documents/uni_work/FYP_limping_bot/pics4report/forwards.png')
#sys.exit()

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
lein = 10
for key,val in forwards.iteritems():
    for k,v in forwards.iteritems():
        if 0<v[0]-val[-1]<lein*len(val):
            fill = [item+val[-1] for item in range(v[0]-val[-1])[1:]]
            forwards[key]=val+fill+v
        elif 0<val[0]-v[-1]<lein*len(val):
            fill = [item+v[-1] for item in range(val[0]-v[-1])[1:]]
            forwards[key]=v+fill+val

#summed = []
#for i in range(len(servo_data['t'])):
#    tot = 0
#    for key in servo_data.keys():
#        if key!='t':
#            tot+=(servo_data[key][i]-start)
#    summed.append(tot)
#print forwards
max_key = max(forwards,key=lambda x: len(set(forwards[x])))
#print forwards[max_key]
times = robot_new['t'][forwards[max_key][0]:forwards[max_key][-1]+1]
to_fft= robot_new['forwards'][forwards[max_key][0]:forwards[max_key][-1]+1]
times = range(len(to_fft))
T=[]
for t in range(len(times))[1:]:
    T.append(times[t]-times[t-1])
T = T[len(T)//2]
#print T
per2find = np.zeros(numServos)
shift2find = np.zeros(numServos)
for ser in range(numServos):
    servo_reg = servo_data[str(ser)][forwards[max_key][0]:forwards[max_key][-1]+1]
    peaks = 0
    troughs = 0
    phase = 1
    for point in range(len(servo_reg)):
        if servo_reg[point] == max(servo_reg):
            if servo_reg[point-1]!=servo_reg[point]:
                #print servo_reg[point]
                peaks+=1
        elif servo_reg[point] == min(servo_reg):
            if servo_reg[point-1]!=servo_reg[point]:
                #print servo_reg[point]
                troughs+=1
        if troughs ==1 and peaks==0:
            phase = -1
    cys = (peaks+troughs)/2
    period = phase*(times[-1]-times[0])/cys
    shift = np.arcsin((servo_reg[0]-start)/amp)
    #print shift
    per2find[ser]=period
    shift2find[ser]=shift
    evened = []
    for i in range(len(servo_reg)):
        val = amp*np.sin(np.pi*2*i*T/period+shift)+start
        evened.append(val)
    #print len(evened),len(servo_reg),len(times)
    #plt.plot(times,evened)
    #plt.plot(times,servo_reg)
    #plt.show()
per2find = [int(x) for x in per2find.tolist()]
#shift2find = [int(x) for x in shift2find.tolist()]
#print per2find,shift2find
f = open('period_shift.h','w')
f.write('#ifndef period_shift_h\n#define period_shift_h\nextern int periods[];\nextern float shifts[];\nconst int numServos = '+str(numServos)+';\n#endif')
f.close()
f = open('period_shift.cpp','w')
f.write('#include "period_shift.h"\nint periods[numServos] = {')

for sh in range(numServos)[:-1]:
    f.write(str(per2find[sh])+', ')
f.write(str(per2find[numServos-1]))
f.write('};\nfloat shifts[numServos] = {')
for sh in range(numServos)[:-1]:
    f.write(str(shift2find[sh])+', ')
f.write(str(shift2find[numServos-1]))
f.write('};\n')
f.close()


##### SELECTING PART OF MOTION GRAPH #####
#to_fft = [item+20 for item in to_fft]
#summed_reg = [item+500 for item in summed_reg]
#plt.subplot(211)
#plt.plot(robot_new['t'],robot_new['forwards'])
#plt.plot(times,to_fft)
#plt.ylabel('Forwards movement (Pixels)')
#plt.subplot(212)
#plt.plot(robot_new['t'],summed)
#plt.plot(times,summed_reg)
#plt.xlabel('Time (Milliseconds)')
#plt.ylabel('Summed Servo values')
#plt.show()
#sys.exit()

#servo_traj = {}
#for ser in range(numServos):
#    servo_traj[str(ser)] = {'shift':[],'period':[]}
#N = len(to_fft)
#T = tot[len(tot)//2]
##print T
##x = np.linspace(0.0,N*T,N)
##y = np.sin(50.0*2.0*np.pi*x)+0.5*np.sin(80.0*2.0*np.pi*x)
#scifft = sci.fft(summed_reg)
#scifft = 2.0/N*np.abs(scifft[0:N//2])
##freqs = fftpack.fftfreq(len(to_fft),T)
##plt.plot(freqs,sci.log10(abs(scifft)))
##plt.plot(xf,scifft)
##plt.show()
##sys.exit()
#
#
#plt.plot(xf,scifft,'r-')
##plt.plot(xf,smoothfft,'g-')
##plt.ylim(0,2)
#plt.show()
#sys.exit()
##print scifft
#for per in periods:
#    if not (1000<per<4000):
#        periods = [x for x in periods if x != per]
#
#test = fft(servo_data['0'])
#servoFFT = 2.0/N*np.abs(test[:N//2])
#servoPer = []
#print servoPer
#plt.plot(xf,servoFFT)
#plt.show()

#filtN = 1
#filtCut = 0.4
#B,A = signal.butter(filtN,filtCut,output='ba')
#smoothfft = signal.filtfilt(B,A,scifft)
#print robot_data['ang']

##print servo_traj
#for key,val in servo_traj.iteritems():
#    for KEY,VAL in val.iteritems():
#        val[KEY] = sum(VAL)/len(VAL)
##print servo_traj
#
#summed = []
#for t in range(len(servo_data['t'])):
#    val = 0
#    for key in range(len(servo_data.keys())-1):
#        val+=servo_data[str(key)][t]
#    summed.append(val)
##print servo_data['t']
##summed = robot_data['x']

#servo_reg = {'t':times}
#for key,val in servo_data.iteritems():
#    servo_reg[key]=sci.fft(val[forwards[max_key][0]:forwards[max_key][-1]+1])
#    freqs = []
#    periods = []
#    for i in range(len(xf))[1:-2]:
#        val = double_diff(scifft,xf,xf,i)
#        if val<0:
#            hi = xf[i]
#            freqs.append(hi)
#            plt.axvline(x=hi)
#            periods.append(round(1.0/hi))
    
    #if key != 't':
    #    plt.plot(servo_reg['t'],servo_reg[key])

#plt.plot(servo_reg['t'],servo_reg['0'])
#plt.plot(servo_reg['t'],servo_reg['1'])
#plt.show()
