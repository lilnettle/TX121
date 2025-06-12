#!/usr/bin/env python3
"""
Простий CRSF тест - відправляє пакети на всі знайдені порти
"""

import serial
import time
import os

def find_all_ports():
    """Знайти всі можливі порти"""
    ports = []
    # Всі можливі UART порти
    candidates = [
        "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3",
        "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3",
        "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3",
        "/dev/ttyAMA0", "/dev/ttyAMA1", "/dev/ttyFIQ0"
    ]
    
    for port in candidates:
        if os.path.exists(port):
            ports.append(port)
            
    return ports

def create_simple_crsf_packet():
    """Створити простий CRSF пакет"""
    # Фіксований пакет з відомими значеннями
    packet = bytearray([
        0xC8,  # Sync byte
        0x18,  # Length (24)
        0x16,  # RC Channels type
        # 22 байти payload (всі канали в центрі)
        0x00, 0x04, 0x20, 0x00, 0x04, 0x20, 0x00, 0x04, 
        0x20, 0x00, 0x04, 0x20, 0x00, 0x04, 0x20, 0x00, 
        0x04, 0x20, 0x00, 0x04, 0x20, 0x00,
        0x00   # CRC (спрощений)
    ])
    
    # Простий CRC
    crc = 0
    for byte in packet[2:-1]:  # Від type до кінця payload
        crc ^= byte
        
    packet[-1] = crc
    return bytes(packet)

def test_port(port):
    """Тест одного порту"""
    try:
        print(f"\n🧪 Testing {port}...")
        
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=0.1
        )
        
        packet = create_simple_crsf_packet()
        
        print(f"   ✅ Port opened")
        print(f"   📡 Sending CRSF packets...")
        
        # Відправити 100 пакетів
        for i in range(100):
            ser.write(packet)
            time.sleep(0.02)  # 50 Hz
            
        print(f"   📦 Sent 100 packets")
        ser.close()
        return True
        
    except Exception as e:
        print(f"   ❌ Failed: {e}")
        return False

def main():
    print("🚀 Simple CRSF Port Scanner")
    print("=" * 40)
    
    ports = find_all_ports()
    
    if not ports:
        print("❌ No UART ports found!")
        return
        
    print(f"📡 Found {len(ports)} ports:")
    for port in ports:
        print(f"   {port}")
    
    print(f"\n🧪 Testing each port...")
    print(f"   Watch Betaflight Receiver tab for movement")
    
    working_ports = []
    
    for port in ports:
        if test_port(port):
            working_ports.append(port)
            
        # Пауза між тестами
        time.sleep(1)
    
    print(f"\n📊 Results:")
    print(f"   Working ports: {len(working_ports)}")
    
    if working_ports:
        print(f"   ✅ Successfully tested:")
        for port in working_ports:
            print(f"      {port}")
        print(f"\n💡 If any channels moved in Betaflight, that port works!")
    else:
        print(f"   ❌ No ports responded")
        print(f"\n🔧 Check:")
        print(f"      1. Physical wiring (TX→RX, GND→GND)")
        print(f"      2. Betaflight Serial Rx configuration")
        print(f"      3. Voltage levels (3.3V compatibility)")

if __name__ == "__main__":
    main()