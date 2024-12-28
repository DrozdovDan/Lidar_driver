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
import argparse

# Constants
FRAME_RATE = 20

# Global variables
devices = [lvx.DeviceItem() for _ in range(pylivox.kMaxLidarCount)]
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
            lvx_file_handler.base_points_handle(data, packet)
            point_packet_list.append(packet)

def on_sample_callback(status, handle, response, data):
    print(f"OnSampleCallback status {status} handle {handle} response {response}")
    if status == pylivox.PyLivoxStatus.StatusSuccess():
        if response != 0:
            devices[handle].device_state = lvx.DeviceState.CONNECT
    elif status == pylivox.PyLivoxStatus.StatusTimeout():
        devices[handle].device_state = lvx.DeviceState.CONNECT

def on_stop_sample_callback(status, handle, response, data):
    return

def on_get_lidar_extrinsic_parameter(status, handle, response, data):
    global is_finish_extrinsic_parameter
    if status == pylivox.PyLivoxStatus.StatusSuccess() and response:
        print(f"OnGetLidarExtrinsicParameter statue {status} handle {handle} response {response.ret_code}")
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
            lvx_file_handler.add_device_info(lidar_info)
            if lvx_file_handler.get_device_info_list_size() == connected_lidar_count:
                is_finish_extrinsic_parameter = True
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
        extrinsic_condition.notify()

def on_device_information(status, handle, ack, data):
    if status != pylivox.PyLivoxStatus.StatusSuccess():
        print(f"Device Query Informations Failed {status}")
    if ack:
        print(f"firm ver: {'.'.join(map(str, ack.firmware_version))}")

def lidar_connect(info):
    handle = info.handle
    pylivox.PyQueryDeviceInformation(handle, on_device_information, None)
    if devices[handle].device_state == lvx.DeviceState.DISCONNECT:
        devices[handle].device_state = lvx.DeviceState.CONNECT 
        devices[handle].info = info

def lidar_disconnect(info):
    devices[info.handle].device_state = lvx.DeviceState.DISCONNECT

def lidar_state_change(info):
    devices[info.handle].info = info

def on_device_info_change(info, event_type):
    if not info:
        return
    print(f"OnDeviceChange broadcast code {info.broadcast_code} update type {event_type}")
    handle = info.handle
    if handle >= pylivox.kMaxLidarCount:
        return

    if event_type == pylivox.PyDeviceEvent.EventConnect():
        lidar_connect(info)
        print(f'[WARNING] Lidar sn: [{info.broadcast_code}] Connect!!!')
    elif event_type == pylivox.PyDeviceEvent.EventDisconnect():
        lidar_disconnect(info)
        print(f'[WARNING] Lidar sn: [{info.broadcast_code}] Disconnect!!!')
    elif event_type == pylivox.PyDeviceEvent.EventStateChange():
        lidar_state_change(info)
        print(f'[WARNING] Lidar sn: [{info.broadcast_code}] StateChange!!!')

    if devices[handle].device_state == lvx.DeviceState.CONNECT:
        print(f"Device Working State {devices[handle].info.state}")
        if devices[handle].info.state == pylivox.PyLidarState.LidarStateInit():
            print(f"Device State Change Progress {devices[handle].info.status.progress}")
        else:
            print(f"Device State Error Code 0X{devices[handle].info.status.status_code.error_code}")
        print(f"Device feature {devices[handle].info.feature}")
        pylivox.PySetErrorMessageCallback(handle, on_lidar_error_status_callback)
        if devices[handle].info.state == pylivox.PyLidarState.LidarStateNormal():
            if not is_read_extrinsic_from_xml:
                pylivox.PyLidarGetExtrinsicParameter(handle, on_get_lidar_extrinsic_parameter, None)
            else:
                lidar_get_extrinsic_from_xml(handle) 
            status = pylivox.PyLidarStartSampling(handle, on_sample_callback, None)
            print(f"SAMPLING STATUS: {status}")
            devices[handle].device_state = lvx.DeviceState.SAMPLING

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
    with extrinsic_condition:
        extrinsic_condition.wait()

def add_devices_to_connect():
    global connected_lidar_count
    for code in broadcast_code_rev:
        if broadcast_code_list and code not in broadcast_code_list:
            continue
        handle = 0
        result, handle = pylivox.PyAddLidarToConnect(code, handle)
        if result == pylivox.PyLivoxStatus.StatusSuccess():
            pylivox.PySetDataCallback(handle, get_lidar_data, None)
            devices[handle].handle = handle
            devices[handle].device_state = lvx.DeviceState.DISCONNECT
            connected_lidar_count += 1      

def main():
    global lvx_file_save_time, is_read_extrinsic_from_xml, point_packet_list
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

    print("Start initialize lvx file.")
    if not lvx_file_handler.init_lvx_file():
        pylivox.PyUninit()
        return -1

    lvx_file_handler.init_lvx_file_header()

    last_time = time.monotonic()
    for i in range(lvx_file_save_time * FRAME_RATE):
        point_packet_list_temp = deque()
        with point_pack_condition:
            point_pack_condition.wait(lvx.kDefaultFrameDurationTime / 1000.0 - (time.monotonic() - last_time) / 1000.0)
        last_time = time.monotonic()
        point_packet_list_temp = point_packet_list
        point_packet_list = deque()

        if not point_packet_list_temp:
            print("Point cloud packet is empty.")
            break

        print(f'Finish save {i} frame to lvx file.')
        lvx_file_handler.save_frame_to_lvx_file(point_packet_list_temp)

    lvx_file_handler.close_lvx_file()
    for device in devices:
        if device and device.device_state == lvx.DeviceState.SAMPLING:
            # Stop the sampling of Livox LiDAR.
            pylivox.PyLidarStopSampling(device.handle, on_stop_sample_callback, None)

    # Uninitialize Livox-SDK.
    pylivox.PyUninit()
    return -1

class CodeAction(argparse.Action):

    def __init__(self, option_strings, dest, nargs = None, const = None, default = None, type = None, choices = None, required = False, help = None, metavar = None):
        super().__init__(option_strings, dest, nargs, const, default, type, choices, required, help, metavar)

    def __call__(self, parser, namespace, values, option_string = None):
        global broadcast_code_list
        print(f'Register broadcast code: {values}.')
        broadcast_code_list = values.split('&')

class LogAction(argparse.Action):
    
    def __init__(self, option_strings, dest, nargs = None, const = None, default = None, type = None, choices = None, required = False, help = None, metavar = None):
        super().__init__(option_strings, dest, nargs, const, default, type, choices, required, help, metavar)

    def __call__(self, parser, namespace, values, option_string = None):
        print('Save the log file.')
        pylivox.PySaveLoggerFile()

class TimeAction(argparse.Action):

    def __init__(self, option_strings, dest, nargs = None, const = None, default = None, type = None, choices = None, required = False, help = None, metavar = None):
        super().__init__(option_strings, dest, nargs, const, default, type, choices, required, help, metavar)

    def __call__(self, parser, namespace, values, option_string = None):
        global lvx_file_save_time
        print(f'Time to save point cloud to the lvx file:{values}.')
        lvx_file_save_time = time

class ParamAction(argparse.Action):

    def __init__(self, option_strings, dest, nargs = None, const = None, default = None, type = None, choices = None, required = False, help = None, metavar = None):
        super().__init__(option_strings, dest, nargs, const, default, type, choices, required, help, metavar)

    def __call__(self, parser, namespace, values, option_string = None):
        global is_read_extrinsic_from_xml
        print('Get the extrinsic parameter from extrinsic.xml file.')
        is_read_extrinsic_from_xml = True

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--code', help='Register device broadcast code', type=str, action=CodeAction)
    parser.add_argument('-l', '--log', help='Save the log file', action=LogAction)
    parser.add_argument('-t', '--time', help='Time to save point cloud to the lvx file', type=int, action=TimeAction)
    parser.add_argument('-p', '--param', help='Get the extrinsic parameter from extrinsic.xml file', action=ParamAction)

    main()

