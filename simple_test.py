#!/usr/bin/env python3
"""
RADXA UDP Transmitter з GStreamer OSD
Приймає CRSF з RX приймача, передає через UDP на IP камеру,
запускає GStreamer для відображення RTSP потоку з телеметрією
"""

import serial
import socket
import time
import threading
import logging
import struct
import json
import subprocess
import os
import signal
from pathlib import Path

# Налаштування логування
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('radxa_transmitter.log'),
        logging.StreamHandler()
    ]
)

class RadxaUDPTransmitter:
    def __init__(self, rx_port="/dev/ttyUSB1", camera_ip="192.168.1.100", 
                 crsf_port=5000, telemetry_port=5001, baud=420000, 
                 rtsp_url=None, display_output=":0.0"):
        """
        RADXA UDP передавач з GStreamer OSD
        
        Args:
            rx_port: Порт RX приймача
            camera_ip: IP адреса камери
            crsf_port: UDP порт для CRSF пакетів
            telemetry_port: UDP порт для телеметрії
            baud: Швидкість UART
            rtsp_url: RTSP URL камери (якщо None, то rtsp://camera_ip:554/stream)
            display_output: Display output для GStreamer
        """
        self.rx_port = rx_port
        self.camera_ip = camera_ip
        self.crsf_port = crsf_port
        self.telemetry_port = telemetry_port
        self.baud = baud
        self.rtsp_url = rtsp_url or f"rtsp://{camera_ip}:554/stream"
        self.display_output = display_output
        
        self.rx_serial = None
        self.crsf_socket = None
        self.telemetry_socket = None
        self.gstreamer_process = None
        self.running = False
        
        self.stats = {
            'crsf_sent': 0,
            'telemetry_received': 0,
            'errors': 0,
            'start_time': None
        }
        
        self.rx_buffer = bytearray()
        
        # OSD дані
        self.osd_data = {
            'voltage': 0.0,
            'current': 0.0,
            'battery_percent': 0,
            'rssi': -100,
            'link_quality': 0,
            'armed': False,
            'flight_mode': 'DISARMED',
            'altitude': 0.0,
            'speed': 0.0,
            'timestamp': time.time()
        }
        
        # Шляхи для OSD файлів
        self.osd_dir = Path("/tmp/radxa_osd")
        self.osd_dir.mkdir(exist_ok=True)
        self.osd_json_path = self.osd_dir / "osd_data.json"
        self.osd_overlay_path = self.osd_dir / "overlay.txt"
    
    def connect(self):
        """Підключитися до RX та створити UDP сокети"""
        try:
            # Підключення до RX
            self.rx_serial = serial.Serial(self.rx_port, self.baud, timeout=0.01)
            logging.info(f"✅ Connected to RX at {self.rx_port} @ {self.baud}")
            
            # UDP сокет для відправки CRSF
            self.crsf_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            logging.info(f"✅ CRSF UDP socket created")
            
            # UDP сокет для отримання телеметрії
            self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.telemetry_socket.bind(("0.0.0.0", self.telemetry_port))
            self.telemetry_socket.settimeout(0.1)
            logging.info(f"✅ Telemetry UDP socket bound to port {self.telemetry_port}")
            
            return True
            
        except Exception as e:
            logging.error(f"❌ Connection failed: {e}")
            self.disconnect()
            return False
    
    def disconnect(self):
        """Відключити всі з'єднання"""
        if self.rx_serial:
            self.rx_serial.close()
            self.rx_serial = None
            
        if self.crsf_socket:
            self.crsf_socket.close()
            self.crsf_socket = None
            
        if self.telemetry_socket:
            self.telemetry_socket.close()
            self.telemetry_socket = None
        
        self.stop_gstreamer()
        logging.info("🔌 Disconnected")
    
    def validate_crsf_packet(self, packet):
        """Перевірити валідність CRSF пакета"""
        if len(packet) < 4:
            return False
            
        if packet[0] != 0xC8:
            return False
            
        expected_len = packet[1] + 2
        if len(packet) != expected_len:
            return False
            
        # Перевірити CRC8
        crc = 0
        for byte in packet[2:-1]:
            crc = self.crc8_dvb_s2(crc, byte)
            
        return crc == packet[-1]
    
    def crc8_dvb_s2(self, crc, byte):
        """CRC8 DVB-S2 для CRSF"""
        crc = crc ^ byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc = crc << 1
        return crc & 0xFF
    
    def parse_crsf_packets(self, buffer):
        """Витягти CRSF пакети з буфера"""
        packets = []
        
        while len(buffer) >= 4:
            # Знайти sync byte
            sync_pos = -1
            for i in range(len(buffer)):
                if buffer[i] == 0xC8:
                    sync_pos = i
                    break
            
            if sync_pos == -1:
                buffer.clear()
                break
                
            if sync_pos > 0:
                buffer[:sync_pos] = []
                continue
                
            if len(buffer) < 3:
                break
                
            packet_len = buffer[1] + 2
            if packet_len > 64 or packet_len < 4:
                buffer[:1] = []
                continue
                
            if len(buffer) < packet_len:
                break
                
            packet = bytes(buffer[:packet_len])
            buffer[:packet_len] = []
            
            if self.validate_crsf_packet(packet):
                packets.append(packet)
            else:
                self.stats['errors'] += 1
        
        return packets
    
    def create_udp_packet(self, crsf_data):
        """Створити UDP пакет з CRSF даними"""
        timestamp = int(time.time() * 1000000)  # microseconds
        packet_type = 0x01  # CRSF data
        data_length = len(crsf_data)
        
        header = struct.pack('<QBH', timestamp, packet_type, data_length)
        return header + crsf_data
    
    def parse_telemetry_packet(self, udp_data):
        """Розпарсити телеметрію з UDP пакета"""
        if len(udp_data) < 11:  # Мінімальний header
            return None
            
        timestamp, packet_type, data_length = struct.unpack('<QBH', udp_data[:11])
        
        if packet_type == 0x02 and len(udp_data) >= 11 + data_length:  # Telemetry
            return udp_data[11:11+data_length]
        
        return None
    
    def rx_to_udp_thread(self):
        """Потік: RX → UDP (CRSF пакети)"""
        logging.info("🔄 Started RX→UDP thread")
        
        while self.running:
            try:
                if self.rx_serial.in_waiting > 0:
                    data = self.rx_serial.read(self.rx_serial.in_waiting)
                    self.rx_buffer.extend(data)
                    
                    packets = self.parse_crsf_packets(self.rx_buffer)
                    
                    for packet in packets:
                        # Створити UDP пакет та відправити на камеру
                        udp_packet = self.create_udp_packet(packet)
                        self.crsf_socket.sendto(udp_packet, (self.camera_ip, self.crsf_port))
                        self.stats['crsf_sent'] += 1
                        
                        # Логувати RC пакети
                        if len(packet) >= 3 and packet[2] == 0x16:
                            logging.debug(f"📡 CRSF→UDP: RC packet to {self.camera_ip}:{self.crsf_port}")
                
                time.sleep(0.001)
                
            except Exception as e:
                logging.error(f"❌ RX→UDP error: {e}")
                self.stats['errors'] += 1
                time.sleep(0.01)
    
    def udp_to_display_thread(self):
        """Потік: UDP → Display (телеметрія для OSD)"""
        logging.info("🔄 Started UDP→Display thread")
        
        while self.running:
            try:
                udp_data, addr = self.telemetry_socket.recvfrom(1024)
                
                telemetry_data = self.parse_telemetry_packet(udp_data)
                if telemetry_data:
                    self.stats['telemetry_received'] += 1
                    
                    # Обробити телеметрію для OSD
                    self.process_telemetry_for_osd(telemetry_data)
                    
                    logging.debug(f"📡 Telemetry from {addr}: {len(telemetry_data)} bytes")
                
            except socket.timeout:
                continue
            except Exception as e:
                logging.error(f"❌ UDP→Display error: {e}")
                self.stats['errors'] += 1
                time.sleep(0.01)
    
    def process_telemetry_for_osd(self, telemetry_data):
        """Обробити телеметрію для OSD накладання"""
        try:
            # Розпарсити CRSF телеметрію
            if len(telemetry_data) >= 4:
                packet_type = telemetry_data[2]
                self.osd_data['timestamp'] = time.time()
                
                if packet_type == 0x08:  # Battery
                    if len(telemetry_data) >= 10:
                        voltage = int.from_bytes(telemetry_data[3:5], 'big') / 10.0
                        current = int.from_bytes(telemetry_data[5:7], 'big') / 10.0
                        capacity = (telemetry_data[7] << 16) | (telemetry_data[8] << 8) | telemetry_data[9]
                        percentage = telemetry_data[10] if len(telemetry_data) > 10 else 0
                        
                        self.osd_data.update({
                            'voltage': voltage,
                            'current': current,
                            'capacity': capacity,
                            'battery_percent': percentage
                        })
                
                elif packet_type == 0x14:  # Link Statistics
                    if len(telemetry_data) >= 12:
                        rssi = self.signed_byte(telemetry_data[3])
                        lq = telemetry_data[5]
                        
                        self.osd_data.update({
                            'rssi': rssi,
                            'link_quality': lq
                        })
                
                elif packet_type == 0x02:  # GPS
                    if len(telemetry_data) >= 15:
                        lat = int.from_bytes(telemetry_data[3:7], 'big', signed=True) / 10000000.0
                        lon = int.from_bytes(telemetry_data[7:11], 'big', signed=True) / 10000000.0
                        speed = int.from_bytes(telemetry_data[11:13], 'big') / 100.0
                        altitude = int.from_bytes(telemetry_data[13:15], 'big', signed=True) / 100.0
                        
                        self.osd_data.update({
                            'latitude': lat,
                            'longitude': lon,
                            'speed': speed,
                            'altitude': altitude
                        })
                
                elif packet_type == 0x21:  # Flight mode
                    if len(telemetry_data) >= 6:
                        mode_byte = telemetry_data[3]
                        armed_byte = telemetry_data[4]
                        
                        flight_modes = {
                            0: 'MANUAL', 1: 'ACRO', 2: 'ANGLE', 3: 'HORIZON',
                            4: 'GPS_HOLD', 5: 'GPS_HOME', 6: 'GPS_CRUISE'
                        }
                        
                        self.osd_data.update({
                            'flight_mode': flight_modes.get(mode_byte, f'MODE_{mode_byte}'),
                            'armed': bool(armed_byte & 0x01)
                        })
                
                # Оновити OSD файли
                self.update_osd_files()
        
        except Exception as e:
            logging.error(f"❌ OSD processing error: {e}")
    
    def signed_byte(self, b):
        """Конвертувати в signed byte"""
        return b - 256 if b >= 128 else b
    
    def update_osd_files(self):
        """Оновити OSD файли для GStreamer"""
        try:
            # JSON файл з повними даними
            with open(self.osd_json_path, 'w') as f:
                json.dump(self.osd_data, f, indent=2)
            
            # Текстовий overlay для GStreamer textoverlay
            overlay_text = self.generate_overlay_text()
            with open(self.osd_overlay_path, 'w') as f:
                f.write(overlay_text)
                
        except Exception as e:
            logging.debug(f"OSD save error: {e}")
    
    def generate_overlay_text(self):
        """Згенерувати текст для накладання на відео"""
        # Перевірити свіжість даних (не старше 2 секунд)
        data_age = time.time() - self.osd_data['timestamp']
        if data_age > 2.0:
            return "NO TELEMETRY"
        
        # Статус батареї
        battery_status = f"🔋 {self.osd_data['voltage']:.1f}V ({self.osd_data['battery_percent']}%)"
        if self.osd_data['voltage'] < 3.3:
            battery_status = f"🔴 {battery_status}"  # Низький заряд
        elif self.osd_data['voltage'] < 3.6:
            battery_status = f"🟡 {battery_status}"  # Середній заряд
        else:
            battery_status = f"🟢 {battery_status}"  # Високий заряд
        
        # Статус зв'язку
        lq_color = "🟢" if self.osd_data['link_quality'] > 80 else "🟡" if self.osd_data['link_quality'] > 50 else "🔴"
        link_status = f"{lq_color} RSSI: {self.osd_data['rssi']}dBm LQ: {self.osd_data['link_quality']}%"
        
        # Статус польоту
        armed_status = "🔴 ARMED" if self.osd_data['armed'] else "🟢 DISARMED"
        flight_info = f"{armed_status} | Mode: {self.osd_data['flight_mode']}"
        
        # GPS та навігація
        gps_info = ""
        if 'altitude' in self.osd_data and self.osd_data['altitude'] != 0:
            gps_info = f"Alt: {self.osd_data['altitude']:.1f}m | Spd: {self.osd_data['speed']:.1f}m/s"
        
        # Час
        current_time = time.strftime("%H:%M:%S", time.localtime())
        
        # Компонувати OSD
        lines = [
            f"📡 RADXA FPV | {current_time}",
            battery_status,
            link_status,
            flight_info
        ]
        
        if gps_info:
            lines.append(gps_info)
        
        return "\n".join(lines)
    
    def start_gstreamer(self):
        """Запустити GStreamer для відображення RTSP з OSD"""
        try:
            # Створити початковий OSD файл
            self.update_osd_files()
            
            # GStreamer pipeline для RTSP з текстовим накладанням
            gst_pipeline = [
                "gst-launch-1.0",
                # RTSP джерело
                "rtspsrc", f"location={self.rtsp_url}",
                "latency=50", "buffer-mode=1", "!",
                
                # Декодування
                "rtph264depay", "!",
                "avdec_h264", "!",
                
                # Конвертація кольорів
                "videoconvert", "!",
                
                # Текстове накладання з динамічним оновленням
                "textoverlay",
                f"text-file={self.osd_overlay_path}",
                "font-desc=Monospace Bold 16",
                "valignment=top",
                "halignment=left",
                "xpad=20",
                "ypad=20",
                "color=0xFFFFFFFF",  # Білий текст
                "outline-color=0x000000FF",  # Чорний контур
                "!",
                
                # Масштабування (опційно)
                "videoscale", "!",
                "video/x-raw,width=1280,height=720", "!",
                
                # Виведення на екран
                "ximagesink",
                f"display-name={self.display_output}",
                "sync=false"
            ]
            
            # Запустити GStreamer процес
            env = os.environ.copy()
            env['DISPLAY'] = self.display_output
            
            self.gstreamer_process = subprocess.Popen(
                gst_pipeline,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Для коректної зупинки
            )
            
            logging.info(f"🎥 GStreamer started: {self.rtsp_url} → {self.display_output}")
            logging.info(f"📺 OSD overlay: {self.osd_overlay_path}")
            return True
            
        except Exception as e:
            logging.error(f"❌ GStreamer start failed: {e}")
            return False
    
    def stop_gstreamer(self):
        """Зупинити GStreamer"""
        if self.gstreamer_process:
            try:
                # Спробувати м'яку зупинку
                os.killpg(os.getpgid(self.gstreamer_process.pid), signal.SIGTERM)
                
                # Дочекатися завершення або примусово вбити
                try:
                    self.gstreamer_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(self.gstreamer_process.pid), signal.SIGKILL)
                    self.gstreamer_process.wait()
                
                logging.info("🎥 GStreamer stopped")
                
            except Exception as e:
                logging.error(f"❌ GStreamer stop error: {e}")
            
            finally:
                self.gstreamer_process = None
    
    def gstreamer_monitor_thread(self):
        """Потік моніторингу GStreamer"""
        while self.running:
            if self.gstreamer_process:
                # Перевірити чи працює процес
                if self.gstreamer_process.poll() is not None:
                    logging.warning("⚠️ GStreamer process died, restarting...")
                    self.stop_gstreamer()
                    time.sleep(2)
                    if self.running:
                        self.start_gstreamer()
            
            time.sleep(5)
    
    def osd_update_thread(self):
        """Потік оновлення OSD файлів"""
        while self.running:
            try:
                # Оновлювати OSD файли кожні 100мс навіть без нових даних
                # для оновлення часу та перевірки свіжості
                self.update_osd_files()
                time.sleep(0.1)
                
            except Exception as e:
                logging.error(f"❌ OSD update error: {e}")
                time.sleep(1)
    
    def stats_thread(self):
        """Потік статистики"""
        while self.running:
            time.sleep(5)
            
            if self.stats['start_time']:
                uptime = time.time() - self.stats['start_time']
                crsf_rate = self.stats['crsf_sent'] / uptime if uptime > 0 else 0
                tel_rate = self.stats['telemetry_received'] / uptime if uptime > 0 else 0
                
                gst_status = "🎥 Running" if self.gstreamer_process and self.gstreamer_process.poll() is None else "❌ Stopped"
                
                logging.info(f"📊 CRSF→Camera: {self.stats['crsf_sent']} ({crsf_rate:.1f}/s) | "
                           f"Telemetry←Camera: {self.stats['telemetry_received']} ({tel_rate:.1f}/s) | "
                           f"Errors: {self.stats['errors']} | GStreamer: {gst_status} | Uptime: {uptime:.0f}s")
    
    def start(self):
        """Запустити передавач"""
        if not self.connect():
            return False
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        # Запустити GStreamer
        if not self.start_gstreamer():
            logging.warning("⚠️ Failed to start GStreamer, continuing without video display")
        
        # Запустити потоки
        self.rx_thread = threading.Thread(target=self.rx_to_udp_thread, daemon=True)
        self.udp_thread = threading.Thread(target=self.udp_to_display_thread, daemon=True)
        self.stats_th = threading.Thread(target=self.stats_thread, daemon=True)
        self.gst_monitor_th = threading.Thread(target=self.gstreamer_monitor_thread, daemon=True)
        self.osd_update_th = threading.Thread(target=self.osd_update_thread, daemon=True)
        
        self.rx_thread.start()
        self.udp_thread.start()
        self.stats_th.start()
        self.gst_monitor_th.start()
        self.osd_update_th.start()
        
        logging.info(f"🚀 RADXA UDP Transmitter with GStreamer OSD started")
        logging.info(f"📡 Sending CRSF to {self.camera_ip}:{self.crsf_port}")
        logging.info(f"📺 Receiving telemetry on port {self.telemetry_port}")
        logging.info(f"🎥 Video stream: {self.rtsp_url}")
        return True
    
    def stop(self):
        """Зупинити передавач"""
        self.running = False
        time.sleep(0.5)
        self.disconnect()
        logging.info("⏹️ RADXA UDP Transmitter stopped")

def main():
    print("📡 RADXA UDP TRANSMITTER + GSTREAMER OSD")
    print("=" * 50)
    print("Sends CRSF packets to IP camera via UDP")
    print("Receives telemetry for OSD overlay")
    print("Displays RTSP stream with telemetry overlay")
    print()
    
    # Конфігурація
    rx_port = input("RX port [/dev/ttyUSB1]: ").strip() or "/dev/ttyUSB1"
    camera_ip = input("Camera IP [192.168.1.100]: ").strip() or "192.168.1.100"
    crsf_port = int(input("CRSF UDP port [5000]: ").strip() or "5000")
    telemetry_port = int(input("Telemetry UDP port [5001]: ").strip() or "5001")
    baud = int(input("UART baud [420000]: ").strip() or "420000")
    
    # RTSP конфігурація
    default_rtsp = f"rtsp://root:12345@192.168.0.100:554/stream1"
    rtsp_url = input(f"RTSP URL [{default_rtsp}]: ").strip() or default_rtsp
    display_output = input("Display output [:0.0]: ").strip() or ":0.0"
    
    print(f"\n🔧 Configuration:")
    print(f"  RX Port: {rx_port}")
    print(f"  Camera IP: {camera_ip}")
    print(f"  CRSF UDP Port: {crsf_port}")
    print(f"  Telemetry UDP Port: {telemetry_port}")
    print(f"  UART Baud: {baud}")
    print(f"  RTSP URL: {rtsp_url}")
    print(f"  Display: {display_output}")
    print()
    
    # Перевірити наявність GStreamer
    try:
        subprocess.run(["gst-launch-1.0", "--version"], 
                      capture_output=True, check=True)
        print("✅ GStreamer detected")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("❌ GStreamer not found! Install with:")
        print("   sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-*")
        print("   Continuing anyway...")
    
    # Створити передавач
    transmitter = RadxaUDPTransmitter(
        rx_port, camera_ip, crsf_port, telemetry_port, baud, 
        rtsp_url, display_output
    )
    
    try:
        if transmitter.start():
            print("✅ Transmitter + GStreamer running! Press Ctrl+C to stop")
            print("📊 Check radxa_transmitter.log for detailed logs")
            print(f"📺 OSD data: {transmitter.osd_json_path}")
            print(f"🎥 Video overlay: {transmitter.osd_overlay_path}")
            print()
            print("OSD Elements:")
            print("  🔋 Battery voltage and percentage")
            print("  📡 RSSI and Link Quality")
            print("  ✈️ Flight mode and armed status")
            print("  🗺️ Altitude and speed (if available)")
            print("  🕐 Current time")
            
            while True:
                time.sleep(1)
        else:
            print("❌ Failed to start transmitter")
    
    except KeyboardInterrupt:
        print("\n🛑 Stopping transmitter and GStreamer...")
        transmitter.stop()
        print("✅ Everything stopped")
    except Exception as e:
        print(f"❌ Error: {e}")
        transmitter.stop()

if __name__ == "__main__":
    main()