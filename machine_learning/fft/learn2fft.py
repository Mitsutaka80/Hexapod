from modules.camera_convert import createXYangT
from modules.servo_convert import create_servo_dict
import argparse
import matplotlib.pyplot as plt
import numpy as np
import sys
import scipy as sci
import scipy.fftpack as fftpack
import scipy.signal as signal
import matplotlib.patches as mpatches

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

#for t in range(len(robot_new['t']))[n:-(n+1)]:
#    val = double_diff(robot_new['forwards'],robot_new['angle'],robot_new['sideways'],t)
#    gra.append(val)
#    if val<0:
#        if str(loc) not in forwards:
#            forwards[str(loc)] = [t]
#        else:
#            forwards[str(loc)].append(t)
#    else:
#        loc+=1

#fig,ax1 = plt.subplots()
#ax2 = ax1.twinx()
#ax1.plot(robot_new['t'],robot_new['forwards'])
#ax2.plot(robot_vel['t'],robot_vel['forwards'],'r-')
#plt.grid()
#plt.show()
#plt.plot(robot_new['t'],robot_new['forwards'])
##plt.plot(robot_diff['t'],robot_data['x'])
##plt.plot(robot_new['t'],robot_new['sideways'])
##plt.plot(robot_diff['t'],robot_data['y'])
#plt.show()
lein = 8
vis = [val+20 for val in robot_new['forwards']]

#fig = plt.figure(figsize=(12,6))
#plt.rc('axes',labelsize=16)
#plt.plot(robot_diff['t'], robot_diff['forwards'],label='$Original$')
#plt.plot(robot_new['t'], vis,label='$Extrapolated$')
for key,val in forwards.iteritems():
    #TESTime = []
    #testy = []
    #for hi in val:
    #    TESTime.append(robot_new['t'][hi])
    #    testy.append(robot_new['forwards'][hi]+40)
    #plt.plot(TESTime,testy,'r',label='$Selected$')
    for k,v in forwards.iteritems():
        if 0<v[0]-val[-1]<lein*len(val):
            fill = [item+val[-1] for item in range(v[0]-val[-1])[1:]]
            forwards[key]=val+fill+v
            #print v[0]-val[-1]
        elif 0<val[0]-v[-1]<lein*len(val):
            fill = [item+v[-1] for item in range(val[0]-v[-1])[1:]]
            forwards[key]=v+fill+val
            #print val[0]-v[-1]
#red_data = mpatches.Patch(color='red',label='$Selected$')
#handles, labels = plt.gca().get_legend_handles_labels()
#labels, ids = np.unique(labels, return_index=True)
#handles = [handles[i] for i in ids]
#plt.legend(handles, labels, loc='lower right')
#plt.ylabel('$Distance\, travelled\, forwards\, (Pixels)$')
#plt.xlabel('$Time\, (ms)$')
#fig.savefig('/Users/cwiseman/Documents/uni_work/FYP_limping_bot/pics4report/forwardsNMAselecta.png')
##plt.legend(loc="lower right")
#plt.show()
#sys.exit()

summed = []
for i in range(len(servo_data['t'])):
    tot = 0
    for key in servo_data.keys():
        if key!='t':
            tot+=(servo_data[key][i]-start)
    summed.append(tot)
#print forwards
max_key = max(forwards,key=lambda x: len(set(forwards[x])))
#print forwards[max_key]
times = robot_new['t'][forwards[max_key][0]:forwards[max_key][-1]+1]
to_fft= robot_new['forwards'][forwards[max_key][0]:forwards[max_key][-1]+1]
#to_fft = [val+40 for val in to_fft]
vis = [val+20 for val in robot_new['forwards']]

#fig = plt.figure(figsize=(12,6))
#plt.rc('axes',labelsize=16)
#plt.plot(robot_diff['t'], robot_diff['forwards'],label='$Original$')
#plt.plot(robot_new['t'], vis,label='$Extrapolated$')
#plt.plot(times,to_fft,label='$Selected$')
#plt.legend(loc='lower right')
#plt.ylabel('$Distance\, travelled\, forwards\, (Pixels)$')
#plt.xlabel('$Time\, (ms)$')
#plt.show()
#fig.savefig('/Users/cwiseman/Documents/uni_work/FYP_limping_bot/pics4report/forwards_selected.png')
#sys.exit()

summed_reg = summed[forwards[max_key][0]:forwards[max_key][-1]+1]
#times = range(len(summed_reg))

tot = []
for t in range(len(times))[1:]:
    tot.append(times[t]-times[t-1])
tot.sort()
N = len(summed_reg)
T = tot[len(tot)//2]
#T = 1
xf = np.linspace(0.0,1.0/(2.0*T),N//2)
sumfft = sci.fft(summed_reg)
sumfft = 2.0/N*np.abs(sumfft[0:N//2])

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

sum_periods = strongest(sumfft,xf,18)
#print sum_periods

per2find = np.zeros(numServos)
shift2find = np.zeros(numServos)
for ser in range(numServos):
    servo_reg = servo_data[str(ser)][forwards[max_key][0]:forwards[max_key][-1]+1]
    servofft = sci.fft(servo_reg)
    servofft = 2.0/N*np.abs(servofft[0:N//2])
    servo_period = strongest(servofft,xf,1)[0]
    closest = min(sum_periods,key=lambda x:abs(x-servo_period))
    #print closest,servo_period
    servo_per2find = 0
    if abs(servo_period-closest)<0.05*closest:
        servo_per2find = servo_period
        #print 'hi'
        #print ser,servo_period
        #print abs(1/xf[p]-closest)
    count = 0
    indiv = {}
    for p in range(len(servo_reg))[1:]:
        if str(count) not in indiv.keys():
            indiv[str(count)]= {'pos':[servo_reg[p-1],servo_reg[p]],'time':[times[p-1],times[p]]}
        else:
            indiv[str(count)]['pos'].append(servo_reg[p])
            indiv[str(count)]['time'].append(times[p])
        if servo_reg[p-1]>start and servo_reg[p]<=start or servo_reg[p]>=start and servo_reg[p-1]<start:
            indiv[str(count)]['pos'].append(servo_reg[p])
            indiv[str(count)]['time'].append(times[p])
            count+=1
    the_pers = []
    av_shift = []
    for key,val in indiv.iteritems():
        n = len(val['pos'])
        seg_fft = sci.fft(val['pos'])
        seg_fft = 2.0/n*np.abs(seg_fft[0:n//2])
        x = np.linspace(0.0,1.0/(2.0*T),n//2) 
        the_per = (val['time'][-1] - val['time'][0])*2
        #if val['pos'][len(val['pos'])//2] <start:
        #    the_per = -the_per
        #print the_per
        if abs(abs(the_per)-servo_per2find)<servo_per2find*0.05:
            #plt.plot(val['time'],val['pos'])
            #print the_per,servo_per2find
            lhs = np.sin((np.asarray(val['pos'])-start)/amp)
            #print val['pos'],lhs
            shifts = []
            for lh in range(len(lhs)):
                shifts.append(lhs[lh]-(2*np.pi*val['time'][lh])/abs(the_per))
            #print ser,the_per,round(sum(shifts)/len(shifts))
            the_pers.append(the_per)
            av_shift.append(sum(shifts)/len(shifts))
    the_pers.sort()
    av_shift.sort()
    #print the_pers,len(the_pers)//2
    #print av_shift,len(av_shift)//2
    try:
        per2find[ser] = the_pers[len(the_pers)//2]
        shift2find[ser] = round(av_shift[len(av_shift)//2])
    except:
        pass
#print per2find.tolist(),shift2find.tolist()
per2find = [int(x) for x in per2find.tolist()]
shift2find = [int(x) for x in shift2find.tolist()]
#print per2find,shift2find
f = open('period_shift.h','w')
f.write('#ifndef period_shift_h\n#define period_shift_h\nextern int periods[];\nextern int shifts[];\nconst int numServos = '+str(numServos)+';\n#endif')
f.close()
f = open('period_shift.cpp','w')
f.write('#include "period_shift.h"\nint periods[numServos] = {')

for sh in range(numServos)[:-1]:
    f.write(str(per2find[sh])+', ')
f.write(str(per2find[numServos-1]))
f.write('};\nint shifts[numServos] = {')
for sh in range(numServos)[:-1]:
    f.write(str(shift2find[sh])+', ')
f.write(str(shift2find[numServos-1]))
f.write('};\n')
f.close()


##### SELECTING PART OF MOTION GRAPH #####
to_fft = [item+20 for item in to_fft]
summed_reg = [item+500 for item in summed_reg]
fig = plt.figure(figsize=(12,6))
plt.rc('axes',labelsize=16)
plt.subplot(211)
plt.plot(robot_new['t'],robot_new['forwards'],'g',label='$Extrapolated$')
plt.plot(times,to_fft,'r',label='$Selected$')
plt.ylabel('$Travelled\, forwards\, (Pixels)$')
plt.legend(loc='lower right')
plt.subplot(212)
plt.plot(robot_new['t'],summed,'g',label='$Extrapolated$')
plt.plot(times,summed_reg,'r',label='$Selected$')
plt.ylabel('$Summed\, Servo\, values\, (\mu s)$')
plt.xlabel('$Time\, (ms)$')
plt.show()
fig.savefig('/Users/cwiseman/Documents/uni_work/FYP_limping_bot/pics4report/forwards_FFTselecta.png')
sys.exit()
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
