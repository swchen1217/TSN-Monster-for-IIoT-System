import threading
import time
from time import sleep
import serial
import sys
import queue
import pandas as pd
import csv
import sys
import struct
import chn_map_process as cmp

CDC_ACM_DATA = 0
CDC_ACM_CHN_UPDATE = 1
URLLC_DATA_INTERVAL = 0.1  # sleep URLLC_DATA_INTERVAL(seconds) between sending data(sync or urllc) to comport
CHA_MAP_UPDATE_INTERVAL = 2
CHN_MAP_UPDATE_OFFSET = 0.2
CDC_ACM_DATA_MAX_SIZE = 256  # maximum data bytes that can tranfer each time

# com_list = ['com17']
com_list = ['com17', 'com18']
com_threads = {}
com_lock = threading.Lock()  # Lock for synchronizing access to com_queue


def read_data(ser, q):
    while (True):
        queue = ser.inWaiting()
        if queue > 0:
            data = str(ser.readline()).split('\'')[1].split(',')
            data.append(ser.port)
            q.put(data)
            # print(data)


def com_port_init(data_queue):
    for com_name in com_list:
        try:
            opened_com = serial.Serial(com_name, 115200, timeout=0.5)
            # assigned read_data task to thread for relative comport and start it immediately
            read_thread = threading.Thread(target=read_data, args=(opened_com, data_queue))
            read_thread.start()
        except serial.SerialException as e:
            print(f"Failed to open COM port {com_name}. Error: {str(e)}")


def check_data(queue):
    num = 0
    count = 0
    tmp = []
    err = 0
    while (True):
        data = tmp.pop(0) if len(tmp) > 0 else queue.get()
        # print(data)
        if data[0].isdigit() and int(data[0]) > num:
            if int(data[0]) == num + 1:
                num = int(data[0])
                count = count + 1
                print('S:', data[0], 'D:', data[2].split('\\')[0], 'C:', count, 'L:', data[1], 'E:', err, 'P:', data[3])
            else:
                # pass
                print('Waiting...', num + 1)
                tmp.append(data)
                next = []
                for i in range(5):
                    next = queue.get()
                    if (next == num + 1):
                        tmp.insert(0, next)
                        break
                    tmp.append(next)
                err = err + 1
                num = num + 1
                # print(tmp)


def main():
    data_queue = queue.Queue(1000)
    com_port_init(data_queue)
    check_data(data_queue)


if __name__ == '__main__':
    main()
