import csv
import math

def createXYangT(camera_data):
    f = open(camera_data,'rb')
    reader = csv.reader(f)
    csv_list = []
    csv_dict = {}
    for row in reader:
        csv_list.append(row)
    f.close()
    for i in range(len(csv_list[0])):
        csv_dict[csv_list[0][i]] = []
        for row in csv_list[1:]:
            try:
                csv_dict[csv_list[0][i]].append(float(row[i]))
            except:
                csv_dict[csv_list[0][i]].append(None)
    #print csv_dict
    to_pop = []
    for key in csv_dict.keys():
        for i in range(len(csv_dict[key])):
            if csv_dict[key][i] == None:
                if i not in to_pop:
                    to_pop.append(i)
    to_pop.sort()
    #print to_pop
    if len(to_pop)!=0:
        if to_pop[0]!=0:
            to_pop = range(to_pop[0])+to_pop
        for to in to_pop:
            for key in csv_dict.keys():
                del csv_dict[key][0]
    
    difference = []
    test = []
    for ln in range(len(csv_dict['BLUE_X']))[1:]:
        val = 0
        for key in csv_dict.keys():
            if key != 'FPS':
                val += abs(csv_dict[key][ln]-csv_dict[key][ln-1])
        test.append(val)
        #print val
        if val > 6:
            difference.append(ln-1)
    #print test
    #print difference
    for i in range(min(difference)-1):
        for key in csv_dict.keys():
            del csv_dict[key][0]
    XYangT = {'x':[0],'y' : [0],'ang' : [0],'t' : [0]} 

    #x = [0]
    #y = [0]
    #ang = [0]
    #t = [0]
    orig_ang = math.atan2((csv_dict['RED_Y'][0]-csv_dict['BLUE_Y'][0]),(csv_dict['RED_X'][0]-csv_dict['BLUE_X'][0]))
    for fr in range(len(csv_dict['BLUE_X']))[1:]:
        diffxblue= csv_dict['BLUE_X'][fr]-csv_dict['BLUE_X'][0]
        diffxred = csv_dict['RED_X'][fr]-csv_dict['RED_X'][0]
        diffyblue= csv_dict['BLUE_Y'][fr]-csv_dict['BLUE_Y'][0]
        diffyred = csv_dict['RED_Y'][fr]-csv_dict['RED_Y'][0]
        diffang = math.atan2((csv_dict['RED_Y'][fr]-csv_dict['BLUE_Y'][fr]),(csv_dict['RED_X'][fr]-csv_dict['BLUE_X'][fr])) - orig_ang
    
        XYangT['x'].append((diffxblue + diffxred)/2)
        XYangT['y'].append((diffyblue + diffyred)/2)
        XYangT['ang'].append(diffang)
        XYangT['t'].append(1000.0/csv_dict['FPS'][fr-1]+(XYangT['t'][fr-1]))
    
    return XYangT
    file_name = camera_data.replace('.csv','_conv.csv')
    new = open(file_name,'w')
    for i in range(len(XYangT['x'])):
        new.write(str(XYangT['t'][i])+', '+str(XYangT['x'][i])+', '+str(XYangT['y'][i])+', '+str(XYangT['ang'][i])+'\n')
    new.close()
