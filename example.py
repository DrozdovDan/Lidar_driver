import threading
import asyncio
import multiprocessing
import time
from datetime import timedelta
from collections import deque
import pylivox
import lvx
import ctypes
import faulthandler

# Constants
FRAME_RATE = 20

# Global variables
devices = [None] * pylivox.kMaxLidarCount
lvx_file_handler = lvx.LvxFileHandle()
point_packet_list = deque()
broadcast_code_rev = []
mtx = threading.Lock()
lidar_arrive_condition = threading.Condition(mtx)
extrinsic_condition = threading.Condition(mtx)
point_pack_condition = threading.Condition(mtx)
lvx_file_save_time = 10
is_finish_extrinsic_parameter = False
is_read_extrinsic_from_xml = False
connected_lidar_count = 0
PyGILState_Ensure = ctypes.pythonapi.PyGILState_Ensure
PyGILState_Release = ctypes.pythonapi.PyGILState_Release

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
            lidar_info.extrinsic_enable = True
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
    if handle >= pylivox.kMaxLidarCount:
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
            broadcast_code_rev.append(info.broadcast_code)
            with lidar_arrive_condition:
                lidar_arrive_condition.notify()


def wait_for_devices_ready():
    device_ready = False
    wait_time = 2  # seconds
    last_time = time.monotonic()

    while not device_ready:
        # Wait for the condition or timeout
        with lidar_arrive_condition:
            lidar_arrive_condition.wait(wait_time) 

        # Check the elapsed time
        elapsed_time = time.monotonic() - last_time
        if elapsed_time + 0.05 >= wait_time:  # Adding 50ms as a buffer
            device_ready = True
        else:
            last_time = time.monotonic()

def wait_for_extrinsic_parameter():
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
    print("Livox SDK initializing.")
    # Initialize Livox-SDK.
    if not pylivox.PyInit():
        return -1
    print("Livox SDK has been initialized.")

    sdk_version = pylivox.PyLivoxSdkVersion()
    pylivox.PyGetLivoxSdkVersion(sdk_version)
    print("Livox SDK version {0}.{1}.{2} .".format(sdk_version.major, sdk_version.minor, sdk_version.patch))

    # Set the callback function receiving broadcast message from Livox LiDAR.
    pylivox.PySetBroadcastCallback(on_device_broadcast)

    # Set the callback function called when device state change, which means connection/disconnection and changing of LiDAR state.
    pylivox.PySetDeviceStateUpdateCallback(on_device_info_change)

    # Start the device discovering routine.
    if not pylivox.PyStart():
        pylivox.PyUninit()
        return -1
    
    print("Start discovering device.")

    wait_for_devices_ready()

    add_devices_to_connect()

    if connected_lidar_count == 0:
        print("No device will be connected.")
        pylivox.PyUninit()
        return -1

    wait_for_extrinsic_parameter()

    if not lvx_file_handler.InitLvxFile():
        pylivox.PyUninit()
        return -1

    lvx_file_handler.InitLvxFileHeader()

    for i in range(lvx_file_save_time * FRAME_RATE):
        with point_pack_condition:
            point_pack_condition.wait(timeout=lvx.kDefaultFrameDurationTime / 1000.0)

    lvx_file_handler.CloseLvxFile()
    for device in devices:
        if device and device["device_state"] == pylivox.kDeviceStateSampling:
            # Stop the sampling of Livox LiDAR.
            pylivox.LidarStopSampling(device["handle"], on_stop_sample_callback, None)

    # Uninitialize Livox-SDK.
    pylivox.PyUninit()
    return -1

if __name__ == "__main__":
    main()

