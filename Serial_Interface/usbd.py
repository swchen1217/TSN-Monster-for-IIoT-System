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
URLLC_DATA_INTERVAL = 0.01  # 0.1 sleep URLLC_DATA_INTERVAL(seconds) between sending data(sync or urllc) to comport
# URLLC_DATA_INTERVAL = 0  # 0.1 sleep URLLC_DATA_INTERVAL(seconds) between sending data(sync or urllc) to comport
CHA_MAP_UPDATE_INTERVAL = 2
CHN_MAP_UPDATE_OFFSET = 0.2
CDC_ACM_DATA_MAX_SIZE = 256  # maximum data bytes that can tranfer each time

CDC_ACM_TS_1 = 11
CDC_ACM_TS_2 = 12

CDC_ACM_CHN_SET = 2

URLLC_DATA_NUMBER = 100000

com_list = ['com11', 'com12', 'com50']
com_threads = {}
com_lock = threading.Lock()  # Lock for synchronizing access to com_queue

start_time = time.time()

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

    elif data_type == CDC_ACM_CHN_SET:
        val = q_data['chn']
        tlv_data_header = struct.pack('BB', data_type, 4)
        tlv_data = struct.pack("I", val)
        com.write(tlv_data_header)
        com.write(tlv_data)

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

def com_port_init():
    for com_name in com_list:
        try:
            opened_com = serial.Serial(com_name, 115200, timeout=0.5)
            write_queue = queue.Queue(0)
            write_thread = threading.Thread(target=write_data, args=(opened_com, write_queue))
            read_queue = queue.Queue(0)
            read_thread = threading.Thread(target=read_data, args=(opened_com, read_queue))
            com_threads[com_name] = {'com': opened_com, 'write': write_thread, 'write_queue': write_queue, 'read': read_thread, 'read_queue': read_queue}
        except serial.SerialException as e:
            print(f"Failed to open COM port {com_name}. Error: {str(e)}")

def dispatch(data):
    for com_thread in com_threads.values():
        threading.Thread(target=write_data, args=(com_thread['com'], data)).start()

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
        if seq_number == 1000:
            dispatch({'type': CDC_ACM_CHN_SET, 'chn': 20})
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
    threading.Thread(target=generate_cdc_acm_data_3).start()


if __name__ == '__main__':
    main()
