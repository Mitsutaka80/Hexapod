import matplotlib.pyplot as plt
import csv
import argparse

def map_sins(list1,list2map):
    new = []
    fact = (max(list1)-min(list1))/(max(list2map)-min(list2map))
    mean1 = sum(list1)/len(list1)
    mean2map = sum(list2map)/len(list2map)
    shift = mean1 - mean2map*fact
    for i in list2map:
        j = fact*i+shift
        new.append(j)
    return new

parser = argparse.ArgumentParser()
parser.add_argument("A")
args = parser.parse_args()

f = open(args.A,'rb')
reader = csv.reader(f)
comp = {}
the_list = []
for row in reader:
    the_list.append(row)
f.close()
for i in range(len(the_list[0])):
    comp[str(i)]=[]
for j in the_list:
    for k in range(len(j)):
        comp[str(k)].append(float(j[k]))
#for ket in comp.keys():
#    comp[ket] = comp[ket][600:800]

#comp['2'] = map_sins(comp['1'],comp['2'])
#comp['4'] = map_sins(comp['3'],comp['4'])
#for pl in range(len(comp.keys()))[1:]:
#    plt.plot(comp['0'],comp[str(pl)], label=str(pl))
#plt.legend()
lim = 20
fig = plt.figure(figsize=(12,6))
plt.rc('axes',labelsize=16)
ax1 = plt.subplot(211)
#ax2 = ax1.twinx()
ax1.plot(comp['0'],comp['1'],label='$Desired\, value$')
ax1.plot(comp['0'],comp['2'],label='$Read\, value$')
ax1.legend(loc='lower right')
plt.ylabel('$Potentiometer\, 0\, readings$')
#ax2.plot(comp['0'],comp['3'],'r-')
#ax2.set_ylim([-lim,lim])

AX1 = plt.subplot(212)
#AX2 = AX1.twinx()
AX1.plot(comp['0'],comp['4'],label='$Desired\, value$')
AX1.plot(comp['0'],comp['5'],label='$Read\, value$')
AX1.legend(loc='lower right')
plt.ylabel('$Potentiometer\, 1\, readings$')
plt.xlabel('$Timestep$')
#AX2.plot(comp['0'],comp['6'],'r-')
#AX2.set_ylim([-lim,lim])
plt.show()
fig.savefig('/Users/cwiseman/Documents/uni_work/FYP_limping_bot/pics4report/pidSin_notTuned.png')
