import threading
import time
from time import sleep
import serial
import queue
import struct
import chn_map_process as cmp
import hashlib
import mDelay

CDC_ACM_DATA = 0
CDC_ACM_CHN_UPDATE = 1
URLLC_DATA_INTERVAL = 0.1  # 0.1 sleep URLLC_DATA_INTERVAL(seconds) between sending data(sync or urllc) to comport
# URLLC_DATA_INTERVAL = 0  # 0.1 sleep URLLC_DATA_INTERVAL(seconds) between sending data(sync or urllc) to comport
CHA_MAP_UPDATE_INTERVAL = 2
CHN_MAP_UPDATE_OFFSET = 0.2
CDC_ACM_DATA_MAX_SIZE = 256  # maximum data bytes that can tranfer each time

CDC_ACM_TS_1 = 11
CDC_ACM_TS_2 = 12

URLLC_DATA_NUMBER = 100000

com_list = ['com11', 'com12', 'com50']
# com_list = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3']
# com_list = ['/dev/ttyACM1']
# com_list = ['com10','com16']
# com_list = ['com10','com9']
# com_list = ['com10','com9','com12']
# com_list = ['com10','com9','com12','com21','com14']
# chn_map = [20, 34, 13, 24, 8, 17, 11, 2, 27, 25, 35, 29, 5, 31, 3, 10, 14, 23, 1, 28, 9, 18, 0, 6, 7, 15, 16, 21, 19, 33, 12, 4, 32, 26, 22, 30]
com_threads = {}
com_lock = threading.Lock()  # Lock for synchronizing access to com_queue

start_time = time.time()

"""write cdc_acm_data or chn_map to related serial port
   Args:
        com:assign comport number to write data to
"""

# def write_data(com, queue):
#     t1 = time.time()
#     while (True):
#         # com.write(struct.pack("I",7)+'1234'.encode())
#         # com.write(struct.pack('BB',1,4))
#         # print(struct.pack('BB',1,4))
#         # sleep(2)
#         # print('running...')
#         if not queue.empty():
#             q_data = queue.get()
#             data_type = q_data['type']
#             if data_type == CDC_ACM_DATA:
#                 val = q_data['data']
#                 valBytes = str(val).encode()
#                 length = len(valBytes)
#                 seq_number = q_data['seq_num']
#                 tlv_data_header = struct.pack('BB', data_type, length + 4)
#                 tlv_data = struct.pack("I", seq_number) + valBytes
#                 com.write(tlv_data_header + tlv_data)
#                 # print(round((time.time()-t1)*1000))
#                 print(com.port, 'TX', seq_number, val, round((time.time() - start_time) * 1000) / 1000)
#                 # print(com.port, 'TX', seq_number, val, round((time.time()-t1)*1000))
#                 # t1 = time.time()
#
#             elif data_type == CDC_ACM_CHN_UPDATE:
#                 chn_map = q_data['chn_map']
#                 print(chn_map)
#                 chn_map_bytes = struct.pack('B' * len(chn_map), *chn_map)
#                 length = len(chn_map_bytes)
#                 tlv_data_header = struct.pack('BB', data_type, length)
#                 com.write(tlv_data_header)
#                 com.write(chn_map_bytes)
#
#             elif data_type == CDC_ACM_TS_1:
#                 tlv_data_header = struct.pack('BB', data_type, 0)
#                 com.write(tlv_data_header)
#                 # com.write(str('aaaa').encode())
#                 print(com.port, 'TX', tlv_data_header)
#
#             elif data_type == CDC_ACM_TS_2:
#                 tlv_data_header = struct.pack('BB', data_type, 0)
#                 com.write(tlv_data_header)
#                 print(com.port, 'TX', tlv_data_header)
#         else:
#             sleep(0.000001)

def write_data(com, q_data):
    data_type = q_data['type']
    if data_type == CDC_ACM_DATA:
        val = q_data['data']
        valBytes = str(val).encode()
        length = len(valBytes)
        seq_number = q_data['seq_num']
        tlv_data_header = struct.pack('BB', data_type, length + 4)
        tlv_data = struct.pack("I", seq_number) + valBytes
        com.write(tlv_data_header + tlv_data)
        # print(round((time.time()-t1)*1000))
        print(com.port, 'TX', seq_number, val, round((time.time() - start_time) * 1000) / 1000)
        # print(com.port, 'TX', seq_number, val, round((time.time()-t1)*1000))
        # t1 = time.time()

    elif data_type == CDC_ACM_CHN_UPDATE:
        chn_map = q_data['chn_map']
        print(chn_map)
        chn_map_bytes = struct.pack('B' * len(chn_map), *chn_map)
        length = len(chn_map_bytes)
        tlv_data_header = struct.pack('BB', data_type, length)
        com.write(tlv_data_header)
        com.write(chn_map_bytes)

    elif data_type == CDC_ACM_TS_1:
        tlv_data_header = struct.pack('BB', data_type, 0)
        com.write(tlv_data_header)
        # com.write(str('aaaa').encode())
        print(com.port, 'TX', tlv_data_header)

    elif data_type == CDC_ACM_TS_2:
        tlv_data_header = struct.pack('BB', data_type, 0)
        com.write(tlv_data_header)
        print(com.port, 'TX', tlv_data_header)


"""read data from related serial port
   Args:
        com:assign comport number to write data to
        q:put data into this queue
"""

def read_data(ser, q):
    while (True):
        # queue = ser.inWaiting()
        # if queue > 0:
        #     data = ser.read_all()
        #     print(ser.port, 'RX', data)
        n = ser.inWaiting()
        # print(n)
        data = ser.read(n)
        if data:
            print(ser.port, 'RX', data, round((time.time() - start_time) * 1000) / 1000)


"""open comports and assigned thread task to them  
"""


def com_port_init():
    # write_thread_assigned_list = []
    # open com and assign thread
    for com_name in com_list:
        try:
            opened_com = serial.Serial(com_name, 115200, timeout=0.5)
            write_queue = queue.Queue(0)
            write_thread = threading.Thread(target=write_data, args=(opened_com, write_queue))
            # write_thread_assigned_list.append(write_thread)
            # assigned read_data task to thread for relative comport and start it immediately
            read_queue = queue.Queue(0)
            read_thread = threading.Thread(target=read_data, args=(opened_com, read_queue))
            com_threads[com_name] = {'com': opened_com, 'write': write_thread, 'write_queue': write_queue, 'read': read_thread, 'read_queue': read_queue}
        except serial.SerialException as e:
            print(f"Failed to open COM port {com_name}. Error: {str(e)}")

    # # start assigned thread
    # for com_thread in com_threads.values():
    #     com_thread['write'].start()
    #     com_thread['read'].start()

def dispatch(data):
    for com_thread in com_threads.values():
        threading.Thread(target=write_data, args=(com_thread['com'], data)).start()
        # print('T:', threading.active_count())
        # w_queue = com_thread['write_queue']
        # w_queue.put(data)
        # pass

def generate_cdc_acm_data_2():
    seq_number = 0
    last = time.time()
    while True:
        now = time.time()
        if now - last >= URLLC_DATA_INTERVAL:
            last = now
            dispatch({'type': CDC_ACM_DATA, 'seq_num': seq_number, 'data': hashlib.md5(str(seq_number).encode()).hexdigest()})
            seq_number += 1
            if seq_number > URLLC_DATA_NUMBER:
                break
    print("Generate Data Done.", round((time.time() - start_time) * 1000) / 1000)


def generate_cdc_acm_data_3():
    seq_number = 0
    while seq_number < URLLC_DATA_NUMBER:
        seq_number += 1
        # dispatch({'type': CDC_ACM_DATA, 'seq_num': seq_number, 'data': hashlib.md5(str(seq_number).encode()).hexdigest()})
        dispatch({'type': CDC_ACM_DATA, 'seq_num': seq_number, 'data': time.time()})
        mDelay.delayNS(URLLC_DATA_INTERVAL * 1000000000)
    print("Generate Data Done.", round((time.time() - start_time) * 1000) / 1000)



def update_chn_map():
    while True:
        current_chn_map = cmp.get_current_chn_map()
        with com_lock:
            for com_info in com_threads.values():
                com_queue = com_info['queue']
                com_queue.put({'type': CDC_ACM_CHN_UPDATE, 'chn_map': current_chn_map})
                sleep(CHN_MAP_UPDATE_OFFSET)
        sleep(CHA_MAP_UPDATE_INTERVAL)


'''generate CDC_ACM data packets with a sequence number, sends them to the queue of each COM port
'''


def generate_cdc_acm_data():
    seq_number = 0
    last = time.time()
    while True:
        if time.time() - last > URLLC_DATA_INTERVAL:
            last = time.time()
            # print(last)

            for com_info in com_threads.values():
                # print(com_info)
                com_queue = com_info['queue']
                com_queue.put({'type': CDC_ACM_DATA, 'seq_num': seq_number,
                               'data': hashlib.md5(str(seq_number).encode()).hexdigest()})
                # print("TX",com_info[''])
            seq_number += 1
            # sleep(URLLC_DATA_INTERVAL)
            if seq_number > 1000:
                break


# def per_generate():
#     seq_number = 0
#     last = time.time()
#     while True:
#         if time.time() - last > URLLC_DATA_INTERVAL:
#             last = time.time()
#             #
#     # for seq_number in range(1, 10001):
#     #         com_threads['com11']['queue'].put({'type': CDC_ACM_DATA, 'seq_num': seq_number, 'data': hashlib.md5(str(seq_number).encode()).hexdigest()})
#     #         com_threads['com12']['queue'].put({'type': CDC_ACM_DATA, 'seq_num': seq_number, 'data': hashlib.md5(str(seq_number).encode()).hexdigest()})
#             for com_info in com_threads.values():
#                 com_queue = com_info['queue']
#                 com_queue.put({'type': CDC_ACM_DATA, 'seq_num': seq_number,
#                                'data': hashlib.md5(str(seq_number).encode()).hexdigest()})
#             seq_number += 1
#             if seq_number >= 10000:
#                 break
#     print("Per Generate Data OK!")


def test_sync():
    with com_lock:
        for com_info in com_threads.values():
            com_queue = com_info['queue']
            com_queue.put({'type': CDC_ACM_TS_2})

def test(self):
    print("now:", round((time.time() - start_time) * 1000) / 1000)
    self.stop()
    # print("now:", time.time() - start_time)

def main():
    com_port_init()
    # chn_update_thread = threading.Thread(target=update_chn_map)
    # chn_update_thread.start()
    # cmp.open_qos_device()


    # generate_cdc_acm_data()
    # test_sync()

    # generate_cdc_acm_data_2()

    # start assigned thread
    # for com_thread in com_threads.values():
    #     com_thread['write'].start()
    #     com_thread['read'].start()

    start_time = time.time()
    # threading.Thread(target=generate_cdc_acm_data_2).start()


    threading.Thread(target=generate_cdc_acm_data_3).start()

    # generate_cdc_acm_data_3()

    # it = mDelay.SetIntervalNS(500000, test)


if __name__ == '__main__':
    main()
