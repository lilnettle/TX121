#!/usr/bin/env python3
"""
Сканер всіх можливих UART портів на FC
"""

import serial
import time

def test_uart_response(port, baudrate=115200):
    """Перевірити чи відповідає UART"""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        
        # MSP запит для перевірки відповіді FC
        msp_request = bytes([0x24, 0x4D, 0x3C, 0x00, 0x66, 0x66])  # MSP_API_VERSION
        ser.write(msp_request)
        time.sleep(0.1)
        
        response = ser.read(100)
        ser.close()
        
        if len(response) > 0:
            return f"✅ Responds with {len(response)} bytes: {response[:10].hex()}..."
        else:
            return "❌ No response"
            
    except Exception as e:
        return f"❌ Error: {e}"

def find_betaflight_uart():
    """Знайти на якому порту Betaflight CLI"""
    
    possible_ports = [
        "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3",
        "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3",
        "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3"
    ]
    
    print("🔍 Scanning for Betaflight FC...")
    print("=" * 40)
    
    for port in possible_ports:
        print(f"Testing {port:15} ... ", end="", flush=True)
        result = test_uart_response(port)
        print(result)
    
    print("\n💡 Tips:")
    print("✅ = FC responds on this port (likely CLI)")
    print("❌ = No FC found")
    print("\nIf FC found on different port than /dev/ttyUSB0,")
    print("you need to use that port for CRSF!")

def test_crsf_on_port(port):
    """Тест CRSF на конкретному порту"""
    print(f"\n🧪 Testing CRSF on {port}")
    print("=" * 30)
    
    # Простий CRSF пакет - всі канали центр
    crsf_packet = bytes([
        0xC8,  # Sync
        0x18,  # Length (24)
        0x16,  # Type
        # 22 bytes payload (channels data) - спрощений центр
        0x00, 0x7C, 0x1F, 0x07, 0xC1, 0xF0, 0x3F, 0x0F, 0x83, 0xE0, 0x7F,
        0x1F, 0x07, 0xC1, 0xF0, 0x3F, 0x0F, 0x83, 0xE0, 0x7F, 0x1F, 0x07,
        0x4C   # CRC
    ])
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.01)
        print(f"✅ Opened {port}")
        
        print("📡 Sending CRSF packets for 5 seconds...")
        start_time = time.time()
        packets_sent = 0
        
        while time.time() - start_time < 5:
            ser.write(crsf_packet)
            packets_sent += 1
            time.sleep(0.02)  # 50 Hz
        
        ser.close()
        print(f"✅ Sent {packets_sent} packets to {port}")
        print("👀 Check Betaflight Receiver tab now!")
        
    except Exception as e:
        print(f"❌ Error on {port}: {e}")

def main():
    print("🔧 UART Port Scanner for Betaflight")
    print("=" * 50)
    
    # Крок 1: Знайти всі порти
    find_betaflight_uart()
    
    # Крок 2: Тест CRSF на кожному порту
    print("\n" + "="*50)
    print("🧪 Testing CRSF on all possible ports...")
    
    possible_ports = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]
    
    for port in possible_ports:
        try:
            test_crsf_on_port(port)
            input(f"⏸️  Tested {port}. Check Receiver tab, then press Enter for next port...")
        except KeyboardInterrupt:
            print("\n⏹️  Stopped")
            break
        except:
            continue

if __name__ == "__main__":
    main()