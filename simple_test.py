#!/usr/bin/env python3
"""
RADXA UDP Transmitter
Приймає CRSF з RX приймача і передає через UDP на IP камеру
"""

import serial
import socket
import time
import threading
import logging
import struct
import json

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
    def __init__(self, rx_port="/dev/ttyUSB1", camera_ip="192.168.0.100", 
                 crsf_port=5000, telemetry_port=5001, baud=420000):
        """
        RADXA UDP передавач
        
        Args:
            rx_port: Порт RX приймача
            camera_ip: IP адреса камери
            crsf_port: UDP порт для CRSF пакетів
            telemetry_port: UDP порт для телеметрії
            baud: Швидкість UART
        """
        self.rx_port = rx_port
        self.camera_ip = camera_ip
        self.crsf_port = crsf_port
        self.telemetry_port = telemetry_port
        self.baud = baud
        
        self.rx_serial = None
        self.crsf_socket = None
        self.telemetry_socket = None
        self.running = False
        
        self.stats = {
            'crsf_sent': 0,
            'telemetry_received': 0,
            'errors': 0,
            'start_time': None
        }
        
        self.rx_buffer = bytearray()
    
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
        # Структура UDP пакета:
        # Header: timestamp(8) + packet_type(1) + data_length(2)
        # Data: CRSF packet
        
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
                
                osd_data = {
                    'timestamp': time.time(),
                    'packet_type': packet_type
                }
                
                if packet_type == 0x08:  # Battery
                    if len(telemetry_data) >= 10:
                        voltage = int.from_bytes(telemetry_data[3:5], 'big') / 10.0
                        current = int.from_bytes(telemetry_data[5:7], 'big') / 10.0
                        capacity = (telemetry_data[7] << 16) | (telemetry_data[8] << 8) | telemetry_data[9]
                        percentage = telemetry_data[10] if len(telemetry_data) > 10 else 0
                        
                        osd_data.update({
                            'voltage': voltage,
                            'current': current,
                            'capacity': capacity,
                            'battery_percent': percentage
                        })
                
                elif packet_type == 0x14:  # Link Statistics
                    if len(telemetry_data) >= 12:
                        rssi = self.signed_byte(telemetry_data[3])
                        lq = telemetry_data[5]
                        
                        osd_data.update({
                            'rssi': rssi,
                            'link_quality': lq
                        })
                
                # Зберегти для OSD (можна записати в файл або shared memory)
                self.save_osd_data(osd_data)
        
        except Exception as e:
            logging.error(f"❌ OSD processing error: {e}")
    
    def signed_byte(self, b):
        """Конвертувати в signed byte"""
        return b - 256 if b >= 128 else b
    
    def save_osd_data(self, osd_data):
        """Зберегти OSD дані для накладання на відео"""
        # Записати в JSON файл для GSstreamer overlay
        try:
            with open('/tmp/osd_data.json', 'w') as f:
                json.dump(osd_data, f)
        except Exception as e:
            logging.debug(f"OSD save error: {e}")
    
    def stats_thread(self):
        """Потік статистики"""
        while self.running:
            time.sleep(5)
            
            if self.stats['start_time']:
                uptime = time.time() - self.stats['start_time']
                crsf_rate = self.stats['crsf_sent'] / uptime if uptime > 0 else 0
                tel_rate = self.stats['telemetry_received'] / uptime if uptime > 0 else 0
                
                logging.info(f"📊 CRSF→Camera: {self.stats['crsf_sent']} ({crsf_rate:.1f}/s) | "
                           f"Telemetry←Camera: {self.stats['telemetry_received']} ({tel_rate:.1f}/s) | "
                           f"Errors: {self.stats['errors']} | Uptime: {uptime:.0f}s")
    
    def start(self):
        """Запустити передавач"""
        if not self.connect():
            return False
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        # Запустити потоки
        self.rx_thread = threading.Thread(target=self.rx_to_udp_thread, daemon=True)
        self.udp_thread = threading.Thread(target=self.udp_to_display_thread, daemon=True)
        self.stats_th = threading.Thread(target=self.stats_thread, daemon=True)
        
        self.rx_thread.start()
        self.udp_thread.start()
        self.stats_th.start()
        
        logging.info(f"🚀 RADXA UDP Transmitter started")
        logging.info(f"📡 Sending CRSF to {self.camera_ip}:{self.crsf_port}")
        logging.info(f"📺 Receiving telemetry on port {self.telemetry_port}")
        return True
    
    def stop(self):
        """Зупинити передавач"""
        self.running = False
        time.sleep(0.1)
        self.disconnect()
        logging.info("⏹️ RADXA UDP Transmitter stopped")

def main():
    print("📡 RADXA UDP TRANSMITTER")
    print("=" * 40)
    print("Sends CRSF packets to IP camera via UDP")
    print("Receives telemetry for OSD overlay")
    print()
    
    # Конфігурація
    rx_port = input("RX port [/dev/ttyUSB1]: ").strip() or "/dev/ttyUSB1"
    camera_ip = input("Camera IP [192.168.1.100]: ").strip() or "192.168.1.100"
    crsf_port = int(input("CRSF UDP port [5000]: ").strip() or "5000")
    telemetry_port = int(input("Telemetry UDP port [5001]: ").strip() or "5001")
    baud = int(input("UART baud [420000]: ").strip() or "420000")
    
    print(f"\n🔧 Configuration:")
    print(f"  RX Port: {rx_port}")
    print(f"  Camera IP: {camera_ip}")
    print(f"  CRSF UDP Port: {crsf_port}")
    print(f"  Telemetry UDP Port: {telemetry_port}")
    print(f"  UART Baud: {baud}")
    print()
    
    # Створити передавач
    transmitter = RadxaUDPTransmitter(rx_port, camera_ip, crsf_port, telemetry_port, baud)
    
    try:
        if transmitter.start():
            print("✅ Transmitter running! Press Ctrl+C to stop")
            print("📊 Check radxa_transmitter.log for detailed logs")
            print("📺 OSD data saved to /tmp/osd_data.json")
            
            while True:
                time.sleep(1)
        else:
            print("❌ Failed to start transmitter")
    
    except KeyboardInterrupt:
        print("\n🛑 Stopping transmitter...")
        transmitter.stop()
        print("✅ Transmitter stopped")
    except Exception as e:
        print(f"❌ Error: {e}")
        transmitter.stop()

if __name__ == "__main__":
    main()