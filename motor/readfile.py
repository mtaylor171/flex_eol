import csv
import matplotlib.pyplot as plt
import numpy as np

x = []
y = [[],[],[],[],[],[],[],[]]
rising_edge = [[],[],[],[],[],[],[],[]]
freq_count = [[],[],[],[],[],[],[],[]]

display_num = ''
fileName = ''

code = [0, 0, 0]

def collect_data():
    with open(fileName, 'r') as csvfile:
        plots = csv.reader(csvfile,delimiter=',')
        next(plots)
        for row in plots:
            x.append(int(row[0]))
            for i in range(1,int(display_num)+1):
                #y.append([])
                y[i-1].append(int(row[i]))
    print(x)

def analyze_data():
    for i in range(0, int(display_num)):
        rising_edge[i] = np.zeros(len(y[i]), dtype = int)
        rising_edge_detect(y[i], i)
        if (i%2 == 0)&(i < 6):
            calculate_rpm(i)
    find_code()

def find_code():
    global code_count
    code_count = np.zeros(len(y[0]), dtype = int)
    for j in range(0, len(y[0])):
        code = [0, 0, 0]
        for k in range(0, 3):
            if y[k][j] > 2500:
                code[k] = 1
            else:
                code[k] = 0
        #print("Code", code)
        #code_count[j] = find_rotation(code, j)
        #print("Time:", x[j], "us; ", "Code:", code, "; Position: ", code_count[j])
        
def find_rotation(code, j):
    if code == [1, 0, 1]:
        return 1
    if code == [0, 0, 1]:
        return 2
    if code == [0, 1, 1]:
        return 3
    if code == [0, 1, 0]:
        return 4
    if code == [1, 1, 0]:
        return 5
    if code == [1, 0, 0]:
        return 6

def rising_edge_detect(data, i):
    trigger_val = 2000
    for j in range(0,len(data)-1):
        if((data[j] < trigger_val) & (data[j+1] >= trigger_val)):
            rising_edge[i][j] = 1

def calculate_rpm(i):
    freq = 0
    t1 = 0
    t2 = 0
    freq_count[i] = np.zeros(len(y[0]))
    for j in range(0, len(y[0]) - 1):
        if(rising_edge[i][j] == 1):
            t2 = x[j]
            freq = (1000000/(t2-t1))/6
            #print("Freq:", freq)
            t1 = t2          
        freq_count[i][j] = freq         
    
def comparator(data, vref):
    if data >= vref:
        return 1
    elif data < vref:
        return 0

def collect_rpm():
    file = open(fileName + '__speedData', 'w')

def graph_data():
    if(display_code == 'c'):
        plt.plot(x,code_count)
        plt.ylabel('Position')
    elif(int(display_num) > 1):
        if display_code == 'f':
            fig,axs = plt.subplots(3)
        else:
            fig,axs = plt.subplots(int(display_num))
        fig.suptitle('ADC Sampling:\n' + fileName)
        for j in range(0,int(display_num)):
            if(display_code == 'n'):
                axs[j].plot(x,y[j])
            if(display_code == 'r'):
                axs[j].plot(x,rising_edge[j])
            if(display_code == 'f'):
                if (j%2 == 0)&(j < 6):
                    axs[j//2].plot(x,freq_count[j])
            if display_code == 'f':
                if (j%2 == 0)&(j < 6):
                    axs[j//2].set(ylabel = 'Signal {} [mV]'.format(j//2))
            else:
                axs[j].set(ylabel = 'Signal {} [mV]'.format(j))
    else:
        if(display_code == 'n'):
            plt.plot(x,y[0])
        if(display_code == 'r'):
            plt.plot(x,rising_edge[0])
        if(display_code == 'f'):
            plt.plot(x,freq_count[0])
        plt.ylabel('Signal')
    plt.xlabel('time [us]')
    plt.show()

if __name__ == "__main__":
    fileName = input("Enter filename:")
    display_num = input("Enter # channels:")
    display_code = input("Enter display code (n = normal data; r = rising edges; f = frequency; code = c: ")
    collect_data()
    analyze_data()
    graph_data()
    #collect_rpm()
    
