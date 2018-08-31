#!/usr/bin/env python
#reference: https://engineersportal.com/blog/2018/8/14/real-time-graphing-in-python
import serial
import time
import csv
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import matplotlib.ticker as ticker

import signal
import sys

i = 0
size = 30
plt.ion()
fig = plt.figure(figsize=(13,6), clear=True)
ax = fig.add_subplot(111)
ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%0.1f'))
f = open("temp_data.csv","w",newline='\n')

#reference: https://engineersportal.com/blog/2018/8/14/real-time-graphing-in-python
def live_plotter(x_vec,y1_data,line1,identifier='',pause_time=0.1):
    global i
    if line1==[]:
        line1, = ax.plot(x_vec,y1_data,'-o',alpha=0.8)        
        plt.ylabel('Temperature Celcius')
        plt.title('{}'.format(identifier))
        plt.show()

    i=i+1
    ax.set_ylim(min(y1_data)-0.2,max(y1_data)+0.2,auto=True)
    # after the figure, axis, and line are created, we only need to update the y-data
    line1.set_ydata(y1_data)
    # adjust limits if new data goes beyond bounds
    if np.min(y1_data)<=line1.axes.get_ylim()[0] or np.max(y1_data)>=line1.axes.get_ylim()[1]:
        plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])
    # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
    plt.pause(pause_time)
    
    # return line so we can update it again in the next iteration
    return line1

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)

    ser = serial.Serial('COM3', 9600)
    ser.flushInput()

    plt.style.use('ggplot')

    x_vec = np.linspace(0,size,size+1)[0:-1]
    y_vec = np.zeros(len(x_vec))
    line1 = []

    print("Real Time Temperature Plotter")
    while True:
        try:
            ser_bytes = ser.readline()
            print('LOG:{}'.format(ser_bytes))
            if b'START_DATA' in ser_bytes:
                break;
        except KeyboardInterrupt:
            break;
        
    while True:
        try:
            ser_bytes = ser.readline()
            try:
                decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
                print('Current Temp: {} Celcius'.format(decoded_bytes), end="\r", flush=True)
            except:
                continue

            writer = csv.writer(f,delimiter=",")
            writer.writerow([i,decoded_bytes])
            
            y_vec[-1] = decoded_bytes
            line1 = live_plotter(x_vec,y_vec,line1, "Temperature Real-Time Data from BLE Server")
            y_vec = np.append(y_vec[1:],0.0)
        except KeyboardInterrupt:
            break;
    

if __name__ == "__main__":
    main()
    
