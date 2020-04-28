from scipy.fftpack import fft
import numpy as np
import matplotlib.pyplot as plt
import sys
import scipy.signal as signal

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

# Number of sample points
N = 600
# sample spacing
T = 1.0 / 800.0
x0 = np.linspace(0, N/2, N/2)
x1 = np.linspace(N/2, N, N/2)
#print x
x = x0.tolist()+x1.tolist()
#print x0,x1
y0 = np.sin(50.0 * 2.0*np.pi*x0*T) 
y1 = 0.5*np.sin(80.0 * 2.0*np.pi*x1*T)
y = y0.tolist()+y1.tolist()
#print y0,y1
x = np.linspace(0,len(y),len(y))
#plt.plot(x,y)
#plt.show()

#print len(x),len(y)
#y=[1]*len(x)
yf = 2.0/N*np.abs(fft(y)[0:N//2])
xf = np.linspace(0.0, 1.0/(T*2.0), N//2)
#print len(xf),len(yf)
N  = 1    # Filter order
Wn = 0.4 # Cutoff frequency
B, A = signal.butter(N, Wn, output='ba')
smooth = signal.filtfilt(B,A, yf)
freqs = []
for i in range(len(smooth))[1:-2]:
    val = double_diff(smooth,xf,xf,i)
    if val<0:
        freqs.append(xf[i])
#print freqs
plt.plot(xf,yf,'g-')
plt.plot(xf,smooth,'r-')
plt.grid()
#plt.xlim([-5,5])
plt.xlabel('Frequency (Hz)')
for x in freqs:
    plt.axvline(x=x)
plt.show()
