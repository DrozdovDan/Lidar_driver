import threading
import time
from collections import deque
import pylivox
import lvx

# Constants
FRAME_RATE = 20

# Global variables
devices = [None] * pylivox.kMaxLidarCount
lvx_file_handler = lvx.LvxFileHandle()
point_packet_list = deque()
broadcast_code_rev = []
lidar_arrive_condition = threading.Condition()
extrinsic_condition = threading.Condition()
point_pack_condition = threading.Condition()
mtx = threading.Lock()
lvx_file_save_time = 10
is_finish_extrinsic_parameter = False
is_read_extrinsic_from_xml = False
connected_lidar_count = 0

broadcast_code_list = []

# Callbacks
def on_lidar_error_status_callback(status, handle, message):
    if message:
        print(f"handle: {handle}")
        print(f"temp_status: {message.lidar_error_code.temp_status}")
        print(f"volt_status: {message.lidar_error_code.volt_status}")
        print(f"motor_status: {message.lidar_error_code.motor_status}")
        print(f"dirty_warn: {message.lidar_error_code.dirty_warn}")
        print(f"firmware_err: {message.lidar_error_code.firmware_err}")
        print(f"pps_status: {message.lidar_error_code.device_status}")
        print(f"fan_status: {message.lidar_error_code.fan_status}")
        print(f"self_heating: {message.lidar_error_code.self_heating}")
        print(f"ptp_status: {message.lidar_error_code.ptp_status}")
        print(f"time_sync_status: {message.lidar_error_code.time_sync_status}")
        print(f"system_status: {message.lidar_error_code.system_status}")

def get_lidar_data(handle, data, data_num, client_data):
    global point_packet_list, connected_lidar_count, is_finish_extrinsic_parameter
    if data and handle < connected_lidar_count and is_finish_extrinsic_parameter:
        with mtx:
            packet = lvx.LvxBasePackDetail()
            packet.device_index = handle
            lvx_file_handler.BasePointsHandle(data, packet)
            point_packet_list.append(packet)

def on_sample_callback(status, handle, response, data):
    print(f"OnSampleCallback status {status} handle {handle} response {response}")
    if status == pylivox.PyLivoxStatus.StatusSuccess():
        if response != 0:
            devices[handle].device_state = lvx.DeviceState.kDeviceStateConnect
    elif status == pylivox.PyLivoxStatus.StatusTimeout():
        devices[handle].device_state = lvx.DeviceState.kDeviceStateConnect

def on_stop_sample_callback(status, handle, response, data):
    pass

def on_get_lidar_extrinsic_parameter(status, handle, response, data):
    global is_finish_extrinsic_parameter
    if status == pylivox.PyLivoxStatus.StatusSuccess() and response:
        print(f"Extrinsic parameters received for handle {handle}")
        with mtx:
            lidar_info = lvx.LvxDeviceInfo()
            lidar_info.device_index = handle
            lidar_info.device_type = devices[handle].info.type
            lidar_info.extrinsic_enable = true
            lidar_info.pitch = response.pitch
            lidar_info.roll = response.roll
            lidar_info.yaw = response.yaw
            lidar_info.x = response.x / 1000
            lidar_info.y = response.y / 1000
            lidar_info.z = response.z / 1000
            lvx_file_handler.AddDeviceInfo(lidar_info)
            if lvx_file_handler.GetDeviceInfoListSize() == connected_lidar_count:
                is_finish_extrinsic_parameter = True
                with extrinsic_condition:
                    extrinsic_condition.notify()
    elif status == pylivox.PyLivoxStatus.StatusTimeout():
        print("GetLidarExtrinsicParameter timeout!")
    
def lidar_get_extrinsic_from_xml(handle):
    global is_finish_extrinsic_parameter
    lidar_info = lvx.LvxDeviceInfo()
    lvx.ParseExtrinsicXml(devices[handle], lidar_info)
    lvx_file_handler.AddDeviceInfo(lidar_info)
    lidar_info.extrinsic_enable = True
    if lvx_file_handler.GetDeviceInfoListSize() == len(broadcast_code_list):
        is_finish_extrinsic_parameter = True
        with extrinsic_condition:
            extrinsic_condition.notify()

def on_device_information(status, handle, ack, data):
    if status != pylivox.PyLivoxStatus.StatusSuccess():
        print(f"Device Query Informations Failed {status}")
    if ack:
        print(f"Firmware version: {'.'.join(map(str, ack.firmware_version))}")

def lidar_connect(info):
    handle = info.handle
    pylivox.PyQueryDeviceInformation(handle, on_device_information, None)
    if devices[handle].device_state == pylivox.PyDeviceState.DeviceStateDisconnect():
        devices[handle].device_state = pylivox.PyDeviceState.DeviceStateConnect() 
        devices[handle].info = info

def lidar_disconnect(info):
    devices[info.handle].device_state = pylivox.PyDeviceState.DeviceStateDisconnect()

def on_device_info_change(info, event_type):
    if not info:
        return
    print(f"OnDeviceChange broadcast code {info.broadcast_code} update type {event_type}")
    handle = info.handle
    if handle >= kMaxLidarCount:
        return

    if event_type == pylivox.PyDeviceEvent.EventConnect():
        lidar_connect(info)
    elif event_type == pylivox.PyDeviceEvent.EventDisconnect():
        lidar_disconnect(info)

    if devices[handle].device_state == pylivox.PyDeviceState.DeviceStateConnect():
        pylivox.PySetErrorMessageCallback(handle, on_lidar_error_status_callback)
        if devices[handle].info.state == pylivox.PyLidarState.LidarStateNormal():
            if not lvx.is_read_extrinsic_from_xml:
                pylivox.PyLidarGetExtrinsicParameter(handle, on_get_lidar_extrinsic_parameter, None)
            else:
                lidar_get_extrinsic_from_xml(handle) 
            pylivox.PyLidarStartSampling(handle, on_sample_callback, None)
            devices[handle].device_state = pylivox.PyDeviceState.DeviceStateSampling()

def on_device_broadcast(info):
    global broadcast_code_rev
    if info and info.dev_type != pylivox.PyDeviceType.DeviceTypeHub():
        print(f"Broadcast code received: {info.broadcast_code}")
        if len(broadcast_code_rev) == 0 or info.broadcast_code not in broadcast_code_rev:
            with lidar_arrive_condition: 
                broadcast_code_rev.append(info.broadcast_code)
                lidar_arrive_condition.notify()

def wait_for_devices_ready():
    with lidar_arrive_condition:
        lidar_arrive_condition.wait(timeout=2)

def wait_for_extrinsic_parameter():
    with extrinsic_condition:
        extrinsic_condition.wait()

def add_devices_to_connect():
    global connected_lidar_count
    for code in broadcast_code_rev:
        if broadcast_code_list and code not in broadcast_code_list:
            continue
        handle = pylivox.PyAddLidarToConnect(code)
        pylivox.PySetDataCallback(handle, get_lidar_data, None)
        devices[handle] = {"handle": handle, "device_state": pylivox.kDeviceStateDisconnect}
        connected_lidar_count += 1

def main():
    global lvx_file_save_time, is_read_extrinsic_from_xml
    pylivox.PyInit()

    pylivox.PySetBroadcastCallback(on_device_broadcast)
    pylivox.PySetDeviceStateUpdateCallback(on_device_info_change)
    pylivox.PyStart()

    wait_for_devices_ready()
    add_devices_to_connect()

    if connected_lidar_count == 0:
        print("No device connected.")
        pylivox.PyUninit()
        return

    wait_for_extrinsic_parameter()

    if not lvx_file_handler.InitLvxFile():
        pylivox.PyUninit()
        return

    lvx_file_handler.InitLvxFileHeader()

    for i in range(lvx_file_save_time * FRAME_RATE):
        with point_pack_condition:
            point_pack_condition.wait(timeout=kDefaultFrameDurationTime / 1000.0)

    lvx_file_handler.CloseLvxFile()
    for device in devices:
        if device and device["device_state"] == pylivox.kDeviceStateSampling:
            pylivox.LidarStopSampling(device["handle"], on_stop_sample_callback, None)

    pylivox.PyUninit()

if __name__ == "__main__":
    main()

