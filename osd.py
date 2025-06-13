#!/usr/bin/env python3
"""
CRSF Bridge з OSD накладанням: USB1 → USB0 + Телеметрія → OSD → HDMI
"""

import serial
import time
import threading
import logging
import subprocess
import os
from dataclasses import dataclass
from typing import Optional, Dict
from enum import IntEnum

# Налаштування логування
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')

@dataclass
class TelemetryData:
    """Структура телеметрії для OSD"""
    voltage: float = 0.0          # Вольтаж батареї (V)
    current: float = 0.0          # Струм (A)
    fuel: int = 0                 # Заряд батареї (%)
    gps_lat: float = 0.0          # GPS широта
    gps_lon: float = 0.0          # GPS довгота
    gps_alt: float = 0.0          # GPS висота (m)
    gps_speed: float = 0.0        # GPS швидкість (km/h)
    gps_sats: int = 0             # Кількість супутників
    rssi: int = 0                 # Сила сигналу
    link_quality: int = 0         # Якість зв'язку (%)
    flight_mode: str = "ACRO"     # Режим польоту
    armed: bool = False           # Озброєння
    failsafe: bool = False        # Failsafe стан

class PacketsTypes(IntEnum):
    """CRSF типи пакетів"""
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
    DEVICE_INFO = 0x29
    CONFIG_READ = 0x2C
    CONFIG_WRITE = 0x2D
    RADIO_ID = 0x3A

def crc8_dvb_s2(crc, a) -> int:
    """CRC8 обчислення для CRSF"""
    crc = crc ^ a
    for ii in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc = crc << 1
    return crc & 0xFF

def crc8_data(data) -> int:
    """CRC8 для масиву даних"""
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc

def crsf_validate_frame(frame) -> bool:
    """Перевірка CRC фрейму"""
    return crc8_data(frame[2:-1]) == frame[-1]

def signed_byte(b):
    """Конвертація в знаковий байт"""
    return b - 256 if b >= 128 else b

class CRSFParser:
    """Покращений CRSF парсер на базі вашого прикладу"""
    
    CRSF_SYNC = 0xC8
    
    def __init__(self):
        self.input_buffer = bytearray()
        self.telemetry = TelemetryData()
    
    def add_data(self, data: bytes) -> bool:
        """Додати дані до буферу та спробувати розпарсити"""
        self.input_buffer.extend(data)
        parsed = False
        
        while len(self.input_buffer) > 2:
            expected_len = self.input_buffer[1] + 2
            if expected_len > 64 or expected_len < 4:
                self.input_buffer = bytearray()
                break
            elif len(self.input_buffer) >= expected_len:
                single_packet = self.input_buffer[:expected_len]
                self.input_buffer = self.input_buffer[expected_len:]
                
                if not crsf_validate_frame(single_packet):
                    packet_hex = ' '.join(map(hex, single_packet))
                    logging.debug(f"CRC error: {packet_hex}")
                else:
                    if self._handle_crsf_packet(single_packet[2], single_packet):
                        parsed = True
            else:
                break
        
        return parsed
    
    def _handle_crsf_packet(self, ptype, data) -> bool:
        """Обробка CRSF пакету (базується на вашому коді)"""
        try:
            if ptype == PacketsTypes.GPS:
                lat = int.from_bytes(data[3:7], byteorder='big', signed=True) / 1e7
                lon = int.from_bytes(data[7:11], byteorder='big', signed=True) / 1e7
                gspd = int.from_bytes(data[11:13], byteorder='big', signed=True) / 36.0  # m/s
                hdg = int.from_bytes(data[13:15], byteorder='big', signed=True) / 100.0
                alt = int.from_bytes(data[15:17], byteorder='big', signed=True) - 1000
                sats = data[17]
                
                self.telemetry.gps_lat = lat
                self.telemetry.gps_lon = lon
                self.telemetry.gps_speed = gspd * 3.6  # конвертація m/s в km/h
                self.telemetry.gps_alt = alt
                self.telemetry.gps_sats = sats
                
                logging.debug(f"GPS: Pos={lat:.6f} {lon:.6f} GSpd={gspd:.1f}m/s Alt={alt}m Sats={sats}")
                
            elif ptype == PacketsTypes.VARIO:
                vspd = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
                logging.debug(f"VSpd: {vspd:.1f}m/s")
                
            elif ptype == PacketsTypes.ATTITUDE:
                pitch = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10000.0
                roll = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10000.0
                yaw = int.from_bytes(data[7:9], byteorder='big', signed=True) / 10000.0
                logging.debug(f"Attitude: Pitch={pitch:.2f} Roll={roll:.2f} Yaw={yaw:.2f} (rad)")
                
            elif ptype == PacketsTypes.BARO_ALT:
                alt = int.from_bytes(data[3:7], byteorder='big', signed=True) / 100.0
                logging.debug(f"Baro Altitude: {alt}m")
                
            elif ptype == PacketsTypes.LINK_STATISTICS:
                rssi1 = signed_byte(data[3])
                rssi2 = signed_byte(data[4])
                lq = data[5]
                snr = signed_byte(data[6])
                
                self.telemetry.rssi = rssi1  # Основний RSSI
                self.telemetry.link_quality = lq
                
                logging.debug(f"RSSI={rssi1}/{rssi2}dBm LQ={lq:03d} SNR={snr}")
                
            elif ptype == PacketsTypes.BATTERY_SENSOR:
                vbat = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
                curr = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10.0
                mah = data[7] << 16 | data[8] << 8 | data[9]
                pct = data[10]
                
                self.telemetry.voltage = vbat
                self.telemetry.current = curr
                self.telemetry.fuel = pct
                
                logging.debug(f"Battery: {vbat:.2f}V {curr:.1f}A {mah}mAh {pct}%")
                
            elif ptype == PacketsTypes.FLIGHT_MODE:
                # Flight mode зазвичай передається як рядок
                try:
                    mode_str = data[3:].decode('utf-8', errors='ignore').strip('\x00')
                    if mode_str:
                        self.telemetry.flight_mode = mode_str
                        logging.debug(f"Flight Mode: {mode_str}")
                except:
                    pass
            
            return True
            
        except Exception as e:
            logging.error(f"CRSF packet parse error: {e}")
            return False

class OSDManager:
    """Менеджер OSD накладання через GStreamer"""
    
    def __init__(self, camera_ip: str = "192.168.0.100"):
        self.camera_ip = camera_ip
        self.telemetry = TelemetryData()
        self.gst_process = None
        self.osd_fifo = "/tmp/osd_overlay.txt"
        self.running = False
        
        # Створити FIFO для передачі OSD тексту
        self._create_fifo()
    
    def _create_fifo(self):
        """Створити FIFO файл для OSD"""
        try:
            if os.path.exists(self.osd_fifo):
                os.unlink(self.osd_fifo)
            os.mkfifo(self.osd_fifo)
            logging.info(f"📄 Created OSD FIFO: {self.osd_fifo}")
        except Exception as e:
            logging.error(f"Failed to create FIFO: {e}")
    
    def update_telemetry(self, telemetry: TelemetryData):
        """Оновити дані телеметрії"""
        self.telemetry = telemetry
        self._update_osd_text()
    
    def _update_osd_text(self):
        """Оновити OSD текст у FIFO"""
        try:
            osd_text = self._format_osd_text()
            with open(self.osd_fifo, 'w', encoding='utf-8') as f:
                f.write(osd_text + '\n')
        except Exception as e:
            logging.debug(f"OSD update error: {e}")
    
    def _format_osd_text(self) -> str:
        """Форматувати OSD текст"""
        # Основна телеметрія
        lines = [
            f"🔋 {self.telemetry.voltage:.1f}V {self.telemetry.fuel}%",
            f"⚡ {self.telemetry.current:.1f}A",
        ]
        
        # RSSI та Link Quality (RSSI зазвичай негативний)
        rssi_display = self.telemetry.rssi if self.telemetry.rssi >= 0 else self.telemetry.rssi
        lines.append(f"📡 RSSI: {rssi_display}dBm LQ: {self.telemetry.link_quality}%")
        
        # Режим польоту
        lines.append(f"🛩️  {self.telemetry.flight_mode} {'🔴ARMED' if self.telemetry.armed else '⚪DISARMED'}")
        
        # GPS інформація
        if self.telemetry.gps_sats > 0:
            lines.extend([
                f"🛰️  GPS: {self.telemetry.gps_sats} sats",
                f"📍 {self.telemetry.gps_lat:.6f}, {self.telemetry.gps_lon:.6f}",
                f"📏 Alt: {self.telemetry.gps_alt:.0f}m Speed: {self.telemetry.gps_speed:.1f}km/h"
            ])
        else:
            lines.append("🛰️  No GPS")
        
        # Попередження
        if self.telemetry.failsafe:
            lines.append("⚠️  FAILSAFE!")
        
        return '\n'.join(lines)
    
    def start_gstreamer(self):
        """Запустити GStreamer pipeline"""
        if self.gst_process:
            return
        
        # GStreamer pipeline з RTSP, OSD накладанням та HDMI виводом
        pipeline = [
            'gst-launch-1.0',
            '-v',
            # RTSP джерело
            f'rtspsrc', f'location=rtsp://{self.camera_ip}:554/stream', 'latency=50', '!',
            'rtph264depay', '!',
            'h264parse', '!',
            'avdec_h264', '!',
            'videoconvert', '!',
            
            # Накладання тексту
            'textoverlay',
            f'text-file={self.osd_fifo}',
            'valignment=top',
            'halignment=left',
            'font-desc="Monospace Bold 14"',
            'color=0xFFFFFFFF',  # Білий текст
            'outline-color=0xFF000000',  # Чорний контур
            'silent=false',
            'auto-resize=false', '!',
            
            # Масштабування для HDMI
            'videoscale', '!',
            'video/x-raw,width=1920,height=1080', '!',
            'videoconvert', '!',
            
            # Вивід на HDMI через kmssink
            'kmssink',
            'connector-id=32',  # Може потребувати налаштування
            'sync=false'
        ]
        
        try:
            logging.info("🎬 Starting GStreamer pipeline...")
            logging.info(f"📺 Camera: rtsp://{self.camera_ip}:554/stream")
            logging.info("🖥️  Output: HDMI via kmssink")
            
            self.gst_process = subprocess.Popen(
                pipeline,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            
            self.running = True
            logging.info("✅ GStreamer started")
            
        except Exception as e:
            logging.error(f"❌ Failed to start GStreamer: {e}")
            self.gst_process = None
    
    def stop_gstreamer(self):
        """Зупинити GStreamer"""
        if self.gst_process:
            self.gst_process.terminate()
            try:
                self.gst_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.gst_process.kill()
            self.gst_process = None
            self.running = False
            logging.info("⏹️ GStreamer stopped")
        
        # Очистити FIFO
        if os.path.exists(self.osd_fifo):
            os.unlink(self.osd_fifo)

class EnhancedCRSFBridge:
    """Розширений CRSF Bridge з OSD функціональністю"""
    
    def __init__(self, camera_ip: str = "192.168.0.100"):
        # Базові налаштування bridge
        self.rx_port = "/dev/ttyUSB1"  # RX приймач
        self.fc_port = "/dev/ttyUSB0"  # FC
        self.baud_rate = 420000
        self.fallback_baud = 115200
        
        self.rx_serial = None
        self.fc_serial = None
        self.running = False
        
        self.stats = {'rx_packets': 0, 'fc_packets': 0, 'errors': 0}
        
        # OSD компоненти
        self.crsf_parser = CRSFParser()
        self.osd_manager = OSDManager(camera_ip)
        self.last_telemetry_update = time.time()
    
    def connect(self):
        """Підключитися з автоматичним fallback"""
        for baud in [self.baud_rate, self.fallback_baud]:
            try:
                logging.info(f"🔌 Trying {baud} baud...")
                
                self.rx_serial = serial.Serial(self.rx_port, baud, timeout=0.01)
                self.fc_serial = serial.Serial(self.fc_port, baud, timeout=0.01)
                
                logging.info(f"✅ Connected at {baud} baud")
                logging.info(f"📡 Bridge: {self.rx_port} → {self.fc_port}")
                return True
                
            except Exception as e:
                logging.error(f"❌ Failed at {baud}: {e}")
                self.disconnect()
        
        return False
    
    def disconnect(self):
        """Відключити"""
        if self.rx_serial:
            self.rx_serial.close()
            self.rx_serial = None
        if self.fc_serial:
            self.fc_serial.close()
            self.fc_serial = None
    
    def bridge_thread(self):
        """Головний потік bridge з парсингом телеметрії"""
        logging.info("🔄 Bridge thread started")
        
        while self.running:
            try:
                # USB1 → USB0 (RX → FC)
                if self.rx_serial.in_waiting > 0:
                    data = self.rx_serial.read(self.rx_serial.in_waiting)
                    self.fc_serial.write(data)
                    self.stats['rx_packets'] += len(data)
                
                # USB0 → USB1 (FC → RX, телеметрія)
                if self.fc_serial.in_waiting > 0:
                    data = self.fc_serial.read(self.fc_serial.in_waiting)
                    self.rx_serial.write(data)
                    self.stats['fc_packets'] += len(data)
                    
                    # Парсинг телеметрії з FC
                    if self.crsf_parser.add_data(data):
                        self.osd_manager.update_telemetry(self.crsf_parser.telemetry)
                        self.last_telemetry_update = time.time()
                
                time.sleep(0.001)  # 1ms
                
            except Exception as e:
                logging.error(f"❌ Bridge error: {e}")
                self.stats['errors'] += 1
                time.sleep(0.01)
    
    def stats_thread(self):
        """Статистика з телеметрією"""
        while self.running:
            time.sleep(5)
            telem = self.crsf_parser.telemetry
            telemetry_age = time.time() - self.last_telemetry_update
            
            logging.info(f"📊 RX→FC: {self.stats['rx_packets']} bytes | "
                        f"FC→RX: {self.stats['fc_packets']} bytes | "
                        f"Errors: {self.stats['errors']}")
            
            if telemetry_age < 5:
                logging.info(f"📡 Telemetry: {telem.voltage:.1f}V, "
                           f"RSSI: {telem.rssi}, LQ: {telem.link_quality}%, "
                           f"GPS: {telem.gps_sats} sats, Mode: {telem.flight_mode}")
            else:
                logging.warning("⚠️  No recent telemetry data")
    
    def start(self):
        """Запустити bridge та OSD"""
        if not self.connect():
            return False
        
        # Запустити GStreamer OSD
        self.osd_manager.start_gstreamer()
        time.sleep(2)  # Дати час GStreamer запуститися
        
        self.running = True
        
        # Запустити потоки
        self.bridge_th = threading.Thread(target=self.bridge_thread, daemon=True)
        self.stats_th = threading.Thread(target=self.stats_thread, daemon=True)
        
        self.bridge_th.start()
        self.stats_th.start()
        
        logging.info("🚀 Enhanced Bridge with OSD running!")
        return True
    
    def stop(self):
        """Зупинити bridge та OSD"""
        self.running = False
        time.sleep(0.1)
        
        # Зупинити OSD
        self.osd_manager.stop_gstreamer()
        
        # Зупинити bridge
        self.disconnect()
        logging.info("⏹️ Enhanced Bridge stopped")

def main():
    print("🌉 ENHANCED CRSF BRIDGE + OSD")
    print("=" * 50)
    print("Configuration:")
    print("  RX Input:     /dev/ttyUSB1")
    print("  FC Output:    /dev/ttyUSB0")
    print("  Camera IP:    192.168.0.100")
    print("  Video Stream: RTSP → HDMI (kmssink)")
    print("  OSD:          CRSF Telemetry Overlay")
    print("  Baud:         420000 (fallback to 115200)")
    print()
    
    # Перевірити чи існують порти
    if not os.path.exists("/dev/ttyUSB0"):
        print("❌ /dev/ttyUSB0 not found!")
        return
    if not os.path.exists("/dev/ttyUSB1"):
        print("❌ /dev/ttyUSB1 not found!")
        return
    
    # Налаштування камери
    camera_ip = input("📷 Camera IP (default: 192.168.0.100): ").strip()
    if not camera_ip:
        camera_ip = "192.168.0.100"
    
    ready = input("❓ Start bridge with OSD? (y/n): ")
    if ready.lower() != 'y':
        return
    
    # Запустити enhanced bridge
    bridge = EnhancedCRSFBridge(camera_ip)
    
    try:
        if bridge.start():
            print("✅ Enhanced Bridge running!")
            print("📺 Video with OSD should appear on HDMI")
            print("📊 Statistics every 5 seconds")
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