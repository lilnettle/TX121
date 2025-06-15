#!/usr/bin/env python3
"""
CRSF Bridge з розпарсингом пакетів
USB1 ↔ USB0 + детальний аналіз телеметрії
"""

import serial
import time
import threading
import logging
from dataclasses import dataclass
from typing import Dict, Optional
from enum import IntEnum

# Налаштування логування
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

CRSF_SYNC = 0xC8

class PacketsTypes(IntEnum):
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    BARO_ALT = 0x09
    HEARTBEAT = 0x0B
    VIDEO_TRANSMITTER = 0x0F
    LINK_STATISTICS = 0x14
    RC_CHANNELS_PACKED = 0x16
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_PING = 0x28
    DEVICE_INFO = 0x29
    PARAMETER_SETTINGS_ENTRY = 0x2B
    PARAMETER_READ = 0x2C
    PARAMETER_WRITE = 0x2D
    RADIO_ID = 0x3A

@dataclass
class CRSFStats:
    rx_to_fc: int = 0
    fc_to_rx: int = 0
    total_frames: int = 0
    telemetry_frames: int = 0
    control_frames: int = 0
    errors: int = 0
    crc_errors: int = 0

class CRSFParser:
    """CRSF пакет парсер"""
    
    @staticmethod
    def crc8_dvb_s2(crc, a) -> int:
        crc = crc ^ a
        for ii in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc = crc << 1
        return crc & 0xFF

    @staticmethod
    def crc8_data(data) -> int:
        crc = 0
        for a in data:
            crc = CRSFParser.crc8_dvb_s2(crc, a)
        return crc

    @staticmethod
    def validate_frame(frame) -> bool:
        if len(frame) < 4:
            return False
        return CRSFParser.crc8_data(frame[2:-1]) == frame[-1]

    @staticmethod
    def signed_byte(b):
        return b - 256 if b >= 128 else b

    @staticmethod
    def parse_packet(frame_data) -> Dict:
        """Розпарсити CRSF пакет"""
        if len(frame_data) < 3:
            return {"error": "Too short"}
        
        ptype = frame_data[2]
        data = frame_data
        result = {"type": ptype, "type_name": "", "parsed": {}}
        
        try:
            if ptype == PacketsTypes.GPS:
                result["type_name"] = "GPS"
                lat = int.from_bytes(data[3:7], byteorder='big', signed=True) / 1e7
                lon = int.from_bytes(data[7:11], byteorder='big', signed=True) / 1e7
                gspd = int.from_bytes(data[11:13], byteorder='big', signed=True) / 36.0
                hdg = int.from_bytes(data[13:15], byteorder='big', signed=True) / 100.0
                alt = int.from_bytes(data[15:17], byteorder='big', signed=True) - 1000
                sats = data[17]
                result["parsed"] = {
                    "latitude": lat,
                    "longitude": lon,
                    "ground_speed": gspd,
                    "heading": hdg,
                    "altitude": alt,
                    "satellites": sats
                }
                
            elif ptype == PacketsTypes.VARIO:
                result["type_name"] = "VARIO"
                vspd = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
                result["parsed"] = {"vertical_speed": vspd}
                
            elif ptype == PacketsTypes.ATTITUDE:
                result["type_name"] = "ATTITUDE"
                pitch = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10000.0
                roll = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10000.0
                yaw = int.from_bytes(data[7:9], byteorder='big', signed=True) / 10000.0
                result["parsed"] = {
                    "pitch": pitch,
                    "roll": roll, 
                    "yaw": yaw
                }
                
            elif ptype == PacketsTypes.BARO_ALT:
                result["type_name"] = "BARO_ALT"
                alt = int.from_bytes(data[3:7], byteorder='big', signed=True) / 100.0
                result["parsed"] = {"altitude": alt}
                
            elif ptype == PacketsTypes.LINK_STATISTICS:
                result["type_name"] = "LINK_STATISTICS"
                rssi1 = CRSFParser.signed_byte(data[3])
                rssi2 = CRSFParser.signed_byte(data[4])
                lq = data[5]
                snr = CRSFParser.signed_byte(data[6])
                ant = data[7] if len(data) > 7 else 0
                rf_mode = data[8] if len(data) > 8 else 0
                tx_power = data[9] if len(data) > 9 else 0
                result["parsed"] = {
                    "rssi1": rssi1,
                    "rssi2": rssi2,
                    "link_quality": lq,
                    "snr": snr,
                    "antenna": ant,
                    "rf_mode": rf_mode,
                    "tx_power": tx_power
                }
                
            elif ptype == PacketsTypes.BATTERY_SENSOR:
                result["type_name"] = "BATTERY_SENSOR"
                vbat = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
                curr = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10.0
                mah = data[7] << 16 | data[8] << 8 | data[9]
                pct = data[10]
                result["parsed"] = {
                    "voltage": vbat,
                    "current": curr,
                    "capacity": mah,
                    "percentage": pct
                }
                
            elif ptype == PacketsTypes.RC_CHANNELS_PACKED:
                result["type_name"] = "RC_CHANNELS_PACKED"
                # Розпакування 16 каналів з 22 байт
                channels = []
                bits = int.from_bytes(data[3:25], 'little')
                for i in range(16):
                    channel = (bits >> (i * 11)) & 0x7FF
                    channels.append(channel)
                result["parsed"] = {"channels": channels}
                
            elif ptype == PacketsTypes.FLIGHT_MODE:
                result["type_name"] = "FLIGHT_MODE"
                mode_str = data[3:].decode('utf-8', errors='ignore').rstrip('\x00')
                result["parsed"] = {"flight_mode": mode_str}
                
            else:
                # Невідомий тип пакету
                result["type_name"] = f"UNKNOWN_{ptype:02X}"
                result["parsed"] = {"raw_data": data[3:].hex()}
                
        except Exception as e:
            result["error"] = str(e)
            
        return result

class EnhancedCRSFBridge:
    def __init__(self):
        self.rx_port = "/dev/ttyUSB1"
        self.fc_port = "/dev/ttyUSB0"
        self.baud_rate = 420000
        self.fallback_baud = 115200
        
        self.rx_serial = None
        self.fc_serial = None
        self.running = False
        
        self.stats = CRSFStats()
        self.debug_mode = True
        self.show_parsed_telemetry = True  # Показувати розпарсену телеметрію
        
        # Буфери для збору фреймів
        self.rx_buffer = bytearray()
        self.fc_buffer = bytearray()
    
    def connect(self):
        """Підключитися з автоматичним fallback"""
        for baud in [self.baud_rate, self.fallback_baud]:
            try:
                logging.info(f"🔌 Connecting at {baud} baud...")
                
                self.rx_serial = serial.Serial(
                    self.rx_port, baud,
                    timeout=0.005,
                    bytesize=8, parity='N', stopbits=1
                )
                
                self.fc_serial = serial.Serial(
                    self.fc_port, baud,
                    timeout=0.005,
                    bytesize=8, parity='N', stopbits=1
                )
                
                # Очистити буфери
                self.rx_serial.flushInput()
                self.rx_serial.flushOutput()
                self.fc_serial.flushInput()
                self.fc_serial.flushOutput()
                
                logging.info(f"✅ Connected at {baud} baud")
                logging.info(f"📡 Bridge: {self.rx_port} ↔ {self.fc_port}")
                return True
                
            except Exception as e:
                logging.error(f"❌ Failed at {baud}: {e}")
                self.disconnect()
        
        return False
    
    def disconnect(self):
        """Відключити"""
        if self.rx_serial:
            try:
                self.rx_serial.close()
            except:
                pass
            self.rx_serial = None
            
        if self.fc_serial:
            try:
                self.fc_serial.close()
            except:
                pass
            self.fc_serial = None
    
    def process_buffer(self, buffer, direction):
        """Обробити буфер і витягти повні фрейми"""
        frames = []
        
        while len(buffer) > 2:
            # Знайти початок фрейму
            if buffer[0] != CRSF_SYNC:
                buffer.pop(0)
                continue
                
            expected_len = buffer[1] + 2
            
            # Перевірити розумність довжини
            if expected_len > 64 or expected_len < 4:
                buffer.pop(0)
                continue
                
            # Чекати повний фрейм
            if len(buffer) < expected_len:
                break
                
            # Витягти фрейм
            frame = bytes(buffer[:expected_len])
            buffer[:expected_len] = []
            
            # Перевірити CRC
            if not CRSFParser.validate_frame(frame):
                self.stats.crc_errors += 1
                logging.warning(f"❌ CRC error in {direction}: {frame.hex()}")
                continue
                
            frames.append(frame)
            
        return frames
    
    def log_parsed_frame(self, frame, direction, parsed_info):
        """Логувати розпарсений фрейм"""
        if not self.show_parsed_telemetry:
            return
            
        type_name = parsed_info.get("type_name", "UNKNOWN")
        parsed_data = parsed_info.get("parsed", {})
        
        if "error" in parsed_info:
            logging.warning(f"{direction}: {type_name} - ERROR: {parsed_info['error']}")
            return
            
        # Форматувати відповідно до типу
        if type_name == "LINK_STATISTICS":
            rssi = parsed_data.get("rssi1", 0)
            lq = parsed_data.get("link_quality", 0)
            snr = parsed_data.get("snr", 0)
            logging.info(f"📊 {direction}: LINK - RSSI={rssi}dBm LQ={lq:03d} SNR={snr}dB")
            
        elif type_name == "BATTERY_SENSOR":
            volt = parsed_data.get("voltage", 0)
            curr = parsed_data.get("current", 0)
            pct = parsed_data.get("percentage", 0)
            logging.info(f"🔋 {direction}: BATTERY - {volt:.2f}V {curr:.1f}A {pct}%")
            
        elif type_name == "GPS":
            lat = parsed_data.get("latitude", 0)
            lon = parsed_data.get("longitude", 0)
            sats = parsed_data.get("satellites", 0)
            alt = parsed_data.get("altitude", 0)
            logging.info(f"🛰️  {direction}: GPS - {lat:.6f},{lon:.6f} Alt={alt}m Sats={sats}")
            
        elif type_name == "ATTITUDE":
            pitch = parsed_data.get("pitch", 0)
            roll = parsed_data.get("roll", 0)
            yaw = parsed_data.get("yaw", 0)
            logging.info(f"🎯 {direction}: ATTITUDE - P={pitch:.2f} R={roll:.2f} Y={yaw:.2f}")
            
        elif type_name == "RC_CHANNELS_PACKED":
            channels = parsed_data.get("channels", [])
            if len(channels) >= 4:
                logging.info(f"🎮 {direction}: RC - CH1-4: {channels[0]} {channels[1]} {channels[2]} {channels[3]}")
            
        elif type_name == "FLIGHT_MODE":
            mode = parsed_data.get("flight_mode", "")
            logging.info(f"✈️  {direction}: MODE - {mode}")
            
        else:
            logging.debug(f"📦 {direction}: {type_name} - {parsed_data}")
    
    def bridge_rx_to_fc(self):
        """RX → FC (команди управління)"""
        while self.running:
            try:
                if self.rx_serial and self.rx_serial.in_waiting > 0:
                    data = self.rx_serial.read(self.rx_serial.in_waiting)
                    self.rx_buffer.extend(data)
                    
                    # Обробити повні фрейми
                    frames = self.process_buffer(self.rx_buffer, "RX→FC")
                    
                    for frame in frames:
                        if self.fc_serial:
                            self.fc_serial.write(frame)
                            self.stats.rx_to_fc += len(frame)
                            self.stats.control_frames += 1
                            
                            # Розпарсити і логувати
                            parsed = CRSFParser.parse_packet(frame)
                            self.log_parsed_frame(frame, "RX→FC", parsed)
                
                time.sleep(0.001)
                
            except Exception as e:
                logging.error(f"❌ RX→FC error: {e}")
                self.stats.errors += 1
                time.sleep(0.01)
    
    def bridge_fc_to_rx(self):
        """FC → RX (телеметрія)"""
        while self.running:
            try:
                if self.fc_serial and self.fc_serial.in_waiting > 0:
                    data = self.fc_serial.read(self.fc_serial.in_waiting)
                    self.fc_buffer.extend(data)
                    
                    # Обробити повні фрейми
                    frames = self.process_buffer(self.fc_buffer, "FC→RX")
                    
                    for frame in frames:
                        if self.rx_serial:
                            self.rx_serial.write(frame)
                            self.stats.fc_to_rx += len(frame)
                            self.stats.telemetry_frames += 1
                            
                            # Розпарсити і логувати
                            parsed = CRSFParser.parse_packet(frame)
                            self.log_parsed_frame(frame, "FC→RX", parsed)
                
                time.sleep(0.001)
                
            except Exception as e:
                logging.error(f"❌ FC→RX error: {e}")
                self.stats.errors += 1
                time.sleep(0.01)
    
    def stats_thread(self):
        """Статистика"""
        while self.running:
            time.sleep(15)  # Кожні 15 секунд
            
            logging.info("=" * 70)
            logging.info(f"📊 BRIDGE STATISTICS:")
            logging.info(f"  RX→FC: {self.stats.rx_to_fc} bytes ({self.stats.control_frames} frames)")
            logging.info(f"  FC→RX: {self.stats.fc_to_rx} bytes ({self.stats.telemetry_frames} frames)")
            logging.info(f"  CRC Errors: {self.stats.crc_errors}")
            logging.info(f"  Other Errors: {self.stats.errors}")
            
            if self.stats.telemetry_frames == 0:
                logging.warning("⚠️  NO TELEMETRY FRAMES! Check FC configuration")
            
            logging.info("=" * 70)
    
    def start(self):
        """Запустити bridge"""
        if not self.connect():
            return False
        
        self.running = True
        
        # Три потоки
        self.rx_to_fc_thread = threading.Thread(target=self.bridge_rx_to_fc, daemon=True)
        self.fc_to_rx_thread = threading.Thread(target=self.bridge_fc_to_rx, daemon=True)
        self.stats_th = threading.Thread(target=self.stats_thread, daemon=True)
        
        self.rx_to_fc_thread.start()
        self.fc_to_rx_thread.start()
        self.stats_th.start()
        
        logging.info("🚀 Enhanced CRSF Bridge with Parser running!")
        logging.info("📡 Parsing and displaying all CRSF packets")
        
        return True
    
    def stop(self):
        """Зупинити bridge"""
        self.running = False
        time.sleep(0.1)
        self.disconnect()
        logging.info("⏹️ Bridge stopped")

def main():
    print("🌉 CRSF BRIDGE WITH PACKET PARSER")
    print("=" * 50)
    print("Features:")
    print("  • Bidirectional: USB1 ↔ USB0")
    print("  • Real-time packet parsing")
    print("  • Detailed telemetry display")
    print("  • CRC validation")
    print("  • Frame statistics")
    print()
    
    # Перевірити порти
    import os
    if not os.path.exists("/dev/ttyUSB0"):
        print("❌ /dev/ttyUSB0 not found!")
        return
    if not os.path.exists("/dev/ttyUSB1"):
        print("❌ /dev/ttyUSB1 not found!")
        return
    
    ready = input("❓ Start CRSF bridge with parser? (y/n): ")
    if ready.lower() != 'y':
        return
    
    # Запустити bridge
    bridge = EnhancedCRSFBridge()
    
    try:
        if bridge.start():
            print("✅ Bridge with parser running!")
            print("📊 Watch parsed telemetry in real-time")
            print("Press Ctrl+C to stop")
            
            while True:
                time.sleep(1)
        else:
            print("❌ Failed to start bridge")
    
    except KeyboardInterrupt:
        print("\n🛑 Stopping...")
        bridge.stop()
        print("✅ Stopped")

if __name__ == "__main__":
    main()