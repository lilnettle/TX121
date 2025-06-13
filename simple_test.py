#!/usr/bin/env python3
"""
Швидкий CRSF Bridge: USB1 → USB0
Без зайвих налаштувань, просто запустити
"""

import serial
import time
import threading
import logging

# Налаштування логування
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')

class SimpleCRSFBridge:
    def __init__(self):
        self.rx_port = "/dev/ttyUSB1"  # RX приймач
        self.fc_port = "/dev/ttyUSB0"  # FC
        self.baud_rate = 420000
        self.fallback_baud = 115200
        
        self.rx_serial = None
        self.fc_serial = None
        self.running = False
        
        self.stats = {'rx_packets': 0, 'fc_packets': 0, 'errors': 0}
    
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
        """Головний потік bridge"""
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
                
                time.sleep(0.001)  # 1ms
                
            except Exception as e:
                logging.error(f"❌ Bridge error: {e}")
                self.stats['errors'] += 1
                time.sleep(0.01)
    
    def stats_thread(self):
        """Статистика"""
        while self.running:
            time.sleep(5)
            logging.info(f"📊 RX→FC: {self.stats['rx_packets']} bytes | "
                        f"FC→RX: {self.stats['fc_packets']} bytes | "
                        f"Errors: {self.stats['errors']}")
    
    def start(self):
        """Запустити bridge"""
        if not self.connect():
            return False
        
        self.running = True
        
        # Запустити потоки
        self.bridge_th = threading.Thread(target=self.bridge_thread, daemon=True)
        self.stats_th = threading.Thread(target=self.stats_thread, daemon=True)
        
        self.bridge_th.start()
        self.stats_th.start()
        
        logging.info("🚀 Bridge running!")
        return True
    
    def stop(self):
        """Зупинити bridge"""
        self.running = False
        time.sleep(0.1)
        self.disconnect()
        logging.info("⏹️ Bridge stopped")

def main():
    print("🌉 QUICK USB1→USB0 CRSF BRIDGE")
    print("=" * 40)
    print("Configuration:")
    print("  RX Input:  /dev/ttyUSB1")
    print("  FC Output: /dev/ttyUSB0")
    print("  Direction: USB1 → USB0")
    print("  Baud: 420000 (fallback to 115200)")
    print()
    
    # Перевірити чи існують порти
    import os
    if not os.path.exists("/dev/ttyUSB0"):
        print("❌ /dev/ttyUSB0 not found!")
        return
    if not os.path.exists("/dev/ttyUSB1"):
        print("❌ /dev/ttyUSB1 not found!")
        return
    
    ready = input("❓ Start bridge? (y/n): ")
    if ready.lower() != 'y':
        return
    
    # Запустити bridge
    bridge = SimpleCRSFBridge()
    
    try:
        if bridge.start():
            print("✅ Bridge running! Press Ctrl+C to stop")
            print("📊 Statistics every 5 seconds")
            
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