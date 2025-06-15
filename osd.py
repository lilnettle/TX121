#!/usr/bin/env python3
"""
Покращений CRSF Bridge з телеметрією
USB1 ↔ USB0 (двосторонній)
"""

import serial
import time
import threading
import logging
from dataclasses import dataclass
from typing import Dict, Optional

# Налаштування логування
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

@dataclass
class CRSFStats:
    rx_to_fc: int = 0      # RX → FC (команди)
    fc_to_rx: int = 0      # FC → RX (телеметрія)
    total_frames: int = 0
    telemetry_frames: int = 0
    control_frames: int = 0
    errors: int = 0

class EnhancedCRSFBridge:
    def __init__(self):
        self.rx_port = "/dev/ttyUSB1"  # RX приймач
        self.fc_port = "/dev/ttyUSB0"  # FC
        self.baud_rate = 420000
        self.fallback_baud = 115200
        
        self.rx_serial = None
        self.fc_serial = None
        self.running = False
        
        self.stats = CRSFStats()
        self.debug_mode = True  # Детальне логування
        
        # CRSF Frame Types для аналізу
        self.CRSF_FRAME_TYPES = {
            0x16: "RC_CHANNELS_PACKED",
            0x14: "LINK_STATISTICS", 
            0x1E: "ATTITUDE",
            0x02: "GPS",
            0x08: "BATTERY_SENSOR",
            0x21: "FLIGHT_MODE",
            0x28: "DEVICE_PING",
            0x29: "DEVICE_INFO",
            0x2B: "PARAMETER_SETTINGS_ENTRY",
            0x2C: "PARAMETER_READ",
            0x2D: "PARAMETER_WRITE"
        }
    
    def connect(self):
        """Підключитися з автоматичним fallback"""
        for baud in [self.baud_rate, self.fallback_baud]:
            try:
                logging.info(f"🔌 Connecting at {baud} baud...")
                
                self.rx_serial = serial.Serial(
                    self.rx_port, baud, 
                    timeout=0.005,  # Менший timeout для швидшої реакції
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
    
    def parse_crsf_frame(self, data: bytes) -> Optional[Dict]:
        """Розібрати CRSF фрейм"""
        if len(data) < 3:
            return None
            
        if data[0] != 0xC8:  # CRSF sync byte
            return None
            
        frame_len = data[1]
        if len(data) < frame_len + 2:
            return None  # Неповний фрейм
            
        frame_type = data[2]
        return {
            'sync': data[0],
            'length': frame_len,
            'type': frame_type,
            'type_name': self.CRSF_FRAME_TYPES.get(frame_type, f"UNKNOWN_{frame_type:02X}"),
            'data': data[3:frame_len+1],
            'full_frame': data[:frame_len+2]
        }
    
    def log_crsf_frame(self, frame_info: Dict, direction: str):
        """Логувати CRSF фрейм"""
        if self.debug_mode:
            hex_data = ' '.join(f'{b:02X}' for b in frame_info['full_frame'])
            logging.debug(f"{direction}: {frame_info['type_name']} | {hex_data}")
    
    def bridge_rx_to_fc(self):
        """RX → FC (команди управління)"""
        while self.running:
            try:
                if self.rx_serial and self.rx_serial.in_waiting > 0:
                    data = self.rx_serial.read(self.rx_serial.in_waiting)
                    
                    if self.fc_serial:
                        self.fc_serial.write(data)
                        self.stats.rx_to_fc += len(data)
                        
                        # Аналіз фрейму
                        frame_info = self.parse_crsf_frame(data)
                        if frame_info:
                            self.stats.control_frames += 1
                            self.log_crsf_frame(frame_info, "RX→FC")
                
                time.sleep(0.001)  # 1ms
                
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
                    
                    if self.rx_serial:
                        self.rx_serial.write(data)
                        self.stats.fc_to_rx += len(data)
                        
                        # Аналіз фрейму
                        frame_info = self.parse_crsf_frame(data)
                        if frame_info:
                            self.stats.telemetry_frames += 1
                            self.log_crsf_frame(frame_info, "FC→RX")
                            
                            # Особливо відмітити телеметрію
                            if frame_info['type'] in [0x14, 0x1E, 0x02, 0x08]:
                                logging.info(f"📊 Telemetry: {frame_info['type_name']}")
                
                time.sleep(0.001)  # 1ms
                
            except Exception as e:
                logging.error(f"❌ FC→RX error: {e}")
                self.stats.errors += 1
                time.sleep(0.01)
    
    def stats_thread(self):
        """Статистика"""
        while self.running:
            time.sleep(10)  # Кожні 10 секунд
            
            logging.info("=" * 60)
            logging.info(f"📊 BRIDGE STATISTICS:")
            logging.info(f"  RX→FC: {self.stats.rx_to_fc} bytes ({self.stats.control_frames} frames)")
            logging.info(f"  FC→RX: {self.stats.fc_to_rx} bytes ({self.stats.telemetry_frames} frames)")
            logging.info(f"  Total: {self.stats.total_frames} frames")
            logging.info(f"  Errors: {self.stats.errors}")
            
            if self.stats.telemetry_frames == 0:
                logging.warning("⚠️  NO TELEMETRY FRAMES! Check FC configuration")
            
            logging.info("=" * 60)
    
    def start(self):
        """Запустити bridge"""
        if not self.connect():
            return False
        
        self.running = True
        
        # Два окремих потоки для кожного напрямку
        self.rx_to_fc_thread = threading.Thread(target=self.bridge_rx_to_fc, daemon=True)
        self.fc_to_rx_thread = threading.Thread(target=self.bridge_fc_to_rx, daemon=True)
        self.stats_th = threading.Thread(target=self.stats_thread, daemon=True)
        
        self.rx_to_fc_thread.start()
        self.fc_to_rx_thread.start()
        self.stats_th.start()
        
        logging.info("🚀 Enhanced Bridge running!")
        logging.info("📡 RX→FC: Control frames")
        logging.info("📊 FC→RX: Telemetry frames")
        
        return True
    
    def stop(self):
        """Зупинити bridge"""
        self.running = False
        time.sleep(0.1)
        self.disconnect()
        logging.info("⏹️ Bridge stopped")

def main():
    print("🌉 ENHANCED CRSF BRIDGE WITH TELEMETRY")
    print("=" * 50)
    print("Bidirectional: USB1 ↔ USB0")
    print("  Control:   RX (USB1) → FC (USB0)")
    print("  Telemetry: FC (USB0) → RX (USB1)")
    print("  Baud: 420000 (fallback to 115200)")
    print()
    
    # Перевірити порти
    import os
    if not os.path.exists("/dev/ttyUSB0"):
        print("❌ /dev/ttyUSB0 not found!")
        return
    if not os.path.exists("/dev/ttyUSB1"):
        print("❌ /dev/ttyUSB1 not found!")
        return
    
    ready = input("❓ Start enhanced bridge? (y/n): ")
    if ready.lower() != 'y':
        return
    
    # Запустити bridge
    bridge = EnhancedCRSFBridge()
    
    try:
        if bridge.start():
            print("✅ Bridge running!")
            print("📊 Detailed statistics every 10 seconds")
            print("🔍 Watch for telemetry frames in logs")
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