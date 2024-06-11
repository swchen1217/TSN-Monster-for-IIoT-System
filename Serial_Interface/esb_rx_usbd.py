import threading
import time
from datetime import datetime

import serial
import queue
import hashlib

CDC_ACM_DATA = 0
CDC_ACM_CHN_UPDATE = 1
URLLC_DATA_INTERVAL = 0.1  # sleep URLLC_DATA_INTERVAL(seconds) between sending data(sync or urllc) to comport
CHA_MAP_UPDATE_INTERVAL = 2
CHN_MAP_UPDATE_OFFSET = 0.2
CDC_ACM_DATA_MAX_SIZE = 256  # maximum data bytes that can tranfer each time

# com_list = ['com17']
com_list = ['com17', 'com18', 'com49']
# com_list = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3']
# com_list = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']
com_threads = {}
com_lock = threading.Lock()  # Lock for synchronizing access to com_queue


def read_data(ser, q):
    buffer = ""
    while (True):
        queue = ser.inWaiting()
        if queue > 0:
            buffer += ser.read_all().decode('utf-8')
            # print(buffer)
            if len(buffer.split(',')) < 4:
                continue
            data = buffer.split(',')
            data.append(ser.port)
            q.put(data)
            # print(data)
            buffer = ""

        # if queue > 0 and len(buffer) > 0 and queue < 128:
        #     data = buffer.split(',')
        #     data.append(ser.port)
        #     # q.put(data)
        #     print(data)
        #     buffer = ""

def com_port_init(data_queue):
    for com_name in com_list:
        try:
            opened_com = serial.Serial(com_name, 115200, timeout=0.5)
            # assigned read_data task to thread for relative comport and start it immediately
            read_thread = threading.Thread(target=read_data, args=(opened_com, data_queue))
            read_thread.start()
        except serial.SerialException as e:
            print(f"Failed to open COM port {com_name}. Error: {str(e)}")


# def check_data(queue):
#     num = 0
#     count = 0
#     tmp = []
#     err = 0
#     t1 = time.time()

#     while (True):
#         data = tmp.pop(0) if len(tmp) > 0 else queue.get()
#         # print(data)
#         if data[0].isdigit() and int(data[0]) > num:
#             if int(data[0]) != num + 1:
#                 print('Lost...', num + 1)
#                 err = err + 1
#             num = int(data[0])
#             count = count + 1
#             if hashlib.md5(str(data[0]).encode()).hexdigest() != data[2]:
#                 err = err + 1
#             print('S:', data[0], 'D:', data[2], 'C:', count, 'L:', data[1], 'E:', err, 'P:', data[4], 'T:', round((time.time()-t1)*1000), 'RSSI:', "-"+str(data[3])+"dbm")
#             t1 = time.time()


def check_data(queue, max_count):
    num = 0
    count = 0
    err = 0
    start_time = time.time()

    c = {'com17':0, 'com18':0, 'com49':0}

    with open(f'log_test_{datetime.now().strftime("%Y-%m-%d-%H-%M-%S")}.txt', 'w') as log_file:
        # while True:
        while count < max_count:
            data = queue.get()
            current_time = time.time()
            # log_file.write(f"{datetime.now()}\n")

            # print(data[4])
            if str(data[4]).startswith("com"):
                c[str(data[4])] += 1

            if data[0].isdigit() and int(data[0]) > num:
                print(time.time() - float(data[2]))
                if int(data[0]) != num + 1:
                    err_msg = f"Lost packet at expected sequence {num + 1}. Actual sequence {data[0]}"
                    print(err_msg)
                    err += 1

                    log_file.write(f"{round((time.time() - start_time) * 1000) / 1000}: {err_msg}\n")
                    log_file.flush()
                num = int(data[0])

                # if str(hashlib.md5(str(data[0]).encode()).hexdigest())*7 != str(data[2]):
                #     err_msg = f"Hash mismatch on sequence {data[0]}"
                #     print(err_msg)
                #     err += 1
                #     log_file.write(f"{round((time.time() - start_time) * 1000) / 1000}: {err_msg}\n")
                #     log_file.flush()

                    # log_file.write(f"{datetime.now()}: {err_msg}\n")
                count += 1
                # print(
                #     f"S: {data[0]}, D: {data[2][:32]}..., C {count}, C1 {c['com17']}, C2 {c['com18']}, C3 {c['com49']}, L: {data[1]}, E: {err}, P: {data[4]}, T: {round((time.time() - start_time) * 1000) / 1000}, RSSI: -{data[3]}dbm")
                print(
                    f"S: {data[0]}, D: {time.time() - float(data[2])}..., C {count}, C1 {c['com17']}, C2 {c['com18']}, C3 {c['com49']}, L: {data[1]}, E: {err}, P: {data[4]}, T: {round((time.time() - start_time) * 1000) / 1000}, RSSI: -{data[3]}dbm")


def main():
    data_queue = queue.Queue(1000)
    com_port_init(data_queue)
    # check_data(data_queue)
    check_data(data_queue, 100000)


if __name__ == '__main__':
    main()
