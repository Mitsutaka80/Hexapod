import csv

def create_servo_dict(servo_csv):
    f = open(servo_csv,'r')
    reader = csv.reader(f)
    comp = {}
    first = 1
    start = 0
    for row in reader:
        if first==1:
            comp['t'] = [0]
            start = float(row[0])
            for i in range(len(row))[:-1]:
                comp[str(i)]=[float(row[i+1])]
            first = 0
        else:
            comp['t'].append(float(row[0])-start)
            for i in range(len(row))[:-1]:
                comp[str(i)].append(float(row[i+1]))

    f.close()
    return comp
