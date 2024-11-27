import time
import os
from typing import List, Union, Any
import pylivox
import struct
from xml.etree import ElementTree as ET

# Constants
WRITE_BUFFER_LEN = 1024 * 1024
MAGIC_CODE = 0xAC0EA767
RAW_POINT_NUM = 100
SINGLE_POINT_NUM = 96
DUAL_POINT_NUM = 48
TRIPLE_POINT_NUM = 30
IMU_POINT_NUM = 1
M_PI = 3.141592653589793

# Constants
kMaxPointSize = 1500
kDefaultFrameDurationTime = 50

# Enumerations
class DeviceState:
    DISCONNECT = 0
    CONNECT = 1
    SAMPLING = 2

# Structures
class DeviceItem:
    def __init__(self, handle: int, device_state: DeviceState, info: pylivox.PyDeviceInfo):
        self.handle = handle
        self.device_state = device_state
        self.info = info

class LvxFilePublicHeader:
    def __init__(self, signature: bytes, version: bytes, magic_code: int):
        self.signature = signature  # 16 bytes
        self.version = version  # 4 bytes
        self.magic_code = magic_code  # 4 bytes

class LvxFilePrivateHeader:
    def __init__(self, frame_duration: int, device_count: int):
        self.frame_duration = frame_duration
        self.device_count = device_count

class LvxDeviceInfo:
    def __init__(self, lidar_broadcast_code: bytes, hub_broadcast_code: bytes, 
                 device_index: int, device_type: int, extrinsic_enable: int,
                 roll: float, pitch: float, yaw: float, x: float, y: float, z: float):
        self.lidar_broadcast_code = lidar_broadcast_code  # 16 bytes
        self.hub_broadcast_code = hub_broadcast_code  # 16 bytes
        self.device_index = device_index
        self.device_type = device_type
        self.extrinsic_enable = extrinsic_enable
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x = x
        self.y = y
        self.z = z

class LvxBasePackDetail:
    def __init__(self, device_index: int, version: int, port_id: int, lidar_index: int, 
                 rsvd: int, error_code: int, timestamp_type: int, data_type: int, 
                 timestamp: bytes, raw_point: bytes, pack_size: int):
        self.device_index = device_index
        self.version = version
        self.port_id = port_id
        self.lidar_index = lidar_index
        self.rsvd = rsvd
        self.error_code = error_code
        self.timestamp_type = timestamp_type
        self.data_type = data_type
        self.timestamp = timestamp  # 8 bytes
        self.raw_point = raw_point  # kMaxPointSize bytes
        self.pack_size = pack_size

class FrameHeader:
    def __init__(self, current_offset: int, next_offset: int, frame_index: int):
        self.current_offset = current_offset
        self.next_offset = next_offset
        self.frame_index = frame_index

class LvxFileHandle:
    def __init__(self):
        self.cur_frame_index = 0
        self.cur_offset = 0
        self.frame_duration = kDefaultFrameDurationTime
        self.lvx_file = None
        self.device_info_list: List[LvxDeviceInfo] = []

    def init_lvx_file(self) -> bool:
        current_time = time.localtime()
        filename = time.strftime("%Y-%m-%d_%H-%M-%S.lvx", current_time)
        try:
            self.lvx_file = open(filename, "wb")
        except IOError:
            return False
        return True
    
    def init_lvx_file_header(self):
        lvx_file_public_header = LvxFilePublicHeader(
            signature=b"livox_tech",
            version=bytes([1, 1, 0, 0]),
            magic_code=MAGIC_CODE
        )

        write_buffer = bytearray(WRITE_BUFFER_LEN)
        struct.pack_into(
            f"{len(lvx_file_public_header.signature)}s4sI",
            write_buffer, self.cur_offset,
            lvx_file_public_header.signature,
            lvx_file_public_header.version,
            lvx_file_public_header.magic_code
        )
        self.cur_offset += struct.calcsize(f"{len(lvx_file_public_header.signature)}s4sI")

        device_count = len(self.device_info_list)
        lvx_file_private_header = LvxFilePrivateHeader(
            frame_duration=self.frame_duration,
            device_count=device_count
        )
        struct.pack_into("I1B", write_buffer, self.cur_offset,
                         lvx_file_private_header.frame_duration,
                         lvx_file_private_header.device_count)
        self.cur_offset += struct.calcsize("I1B")

        for device_info in self.device_info_list:
            struct.pack_into(
                "16s16sBBBBfff",
                write_buffer, self.cur_offset,
                device_info.lidar_broadcast_code,
                device_info.hub_broadcast_code,
                device_info.device_index,
                device_info.device_type,
                device_info.extrinsic_enable,
                device_info.roll,
                device_info.pitch,
                device_info.yaw,
                device_info.x,
                device_info.y,
                device_info.z
            )
            self.cur_offset += struct.calcsize("16s16sBBBBfff")

        self.lvx_file.write(write_buffer[:self.cur_offset])
    
    def save_frame_to_lvx_file(self, point_packet_list_temp: List[LvxBasePackDetail]):
        write_buffer = bytearray(WRITE_BUFFER_LEN)
        cur_pos = 0

        frame_header = FrameHeader(
            current_offset=self.cur_offset,
            next_offset=self.cur_offset + struct.calcsize("QQQ"),
            frame_index=self.cur_frame_index
        )

        for packet in point_packet_list_temp:
            frame_header.next_offset += packet.pack_size

        struct.pack_into("QQQ", write_buffer, cur_pos,
                         frame_header.current_offset,
                         frame_header.next_offset,
                         frame_header.frame_index)
        cur_pos += struct.calcsize("QQQ")

        for packet in point_packet_list_temp:
            if cur_pos + packet.pack_size >= WRITE_BUFFER_LEN:
                self.lvx_file.write(write_buffer[:cur_pos])
                cur_pos = 0
            struct.pack_into(f"{packet.pack_size}s", write_buffer, cur_pos, packet.raw_point)
            cur_pos += packet.pack_size

        self.lvx_file.write(write_buffer[:cur_pos])
        self.cur_offset = frame_header.next_offset
        self.cur_frame_index += 1

    def close_lvx_file(self):
        if self.lvx_file:
            self.lvx_file.close()
            self.lvx_file = None

    def base_points_handle(self, data: Any, packet: LvxBasePackDetail):
        packet.version = data.version
        packet.port_id = data.slot
        packet.lidar_index = data.id
        packet.rsvd = data.rsvd
        packet.error_code = data.err_code
        packet.timestamp_type = data.timestamp_type
        packet.data_type = data.data_type
        packet.timestamp = data.timestamp[:8]

        data_type_map = {
            "kCartesian": (RAW_POINT_NUM, "LivoxRawPoint"),
            "kSpherical": (RAW_POINT_NUM, "LivoxSpherPoint"),
            "kExtendCartesian": (SINGLE_POINT_NUM, "LivoxExtendRawPoint"),
            "kExtendSpherical": (SINGLE_POINT_NUM, "LivoxExtendSpherPoint"),
            "kDualExtendCartesian": (DUAL_POINT_NUM, "LivoxDualExtendRawPoint"),
            "kDualExtendSpherical": (DUAL_POINT_NUM, "LivoxDualExtendSpherPoint"),
            "kTripleExtendCartesian": (TRIPLE_POINT_NUM, "LivoxTripleExtendRawPoint"),
            "kTripleExtendSpherical": (TRIPLE_POINT_NUM, "LivoxTripleExtendSpherPoint"),
            "kImu": (IMU_POINT_NUM, "LivoxImuPoint")
        }

        if packet.data_type in data_type_map:
            point_count, point_type = data_type_map[packet.data_type]
            packet.pack_size = struct.calcsize(f"{point_count}s")
            packet.raw_point = data.data[:packet.pack_size]

def parse_extrinsic_xml(item: DeviceItem, info: LvxDeviceInfo):
    tree = ET.parse("extrinsic.xml")
    root = tree.getroot()

    if root.tag == "Livox":
        for device in root.findall("Device"):
            if device.text == item.info.broadcast_code:
                info.lidar_broadcast_code = device.text.encode()
                info.hub_broadcast_code = b"\x00" * 16
                info.device_type = item.info.type
                info.device_index = item.handle

                for param in device.attrib:
                    value = float(device.attrib[param])
                    setattr(info, param, value)

