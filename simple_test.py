#!/usr/bin/env python3
"""
USB-UART детектор та тестер для CRSF bridge
Допомагає ідентифікувати який порт для RX, а який для FC
"""

import serial
import time
import glob
import threading

def detect_usb_ports():
    """Знайти всі USB-UART порти"""
    ports = sorted(glob.glob('/dev/ttyUSB*'))
    return ports

def get_port_info(port):
    """Отримати інформацію про порт"""
    try:
        # Спробувати різні швидкості
        for baud in [420000, 115200, 57600, 9600]:
            try:
                ser = serial.Serial(port, baud, timeout=0.1)
                ser.close()
                return f"Available at {baud}"
            except:
                continue
        return "No response"
    except Exception as e:
        return f"Error: {e}"

def test_port_for_crsf(port, test_duration=5):
    """Тестувати порт на наявність CRSF трафіку"""
    print(f"🔍 Testing {port} for CRSF traffic...")
    
    crsf_packets = 0
    total_bytes = 0
    
    for baud in [420000, 400000, 416666, 115200]:
        try:
            ser = serial.Serial(port, baud, timeout=0.1)
            print(f"  📡 Listening at {baud} baud...")
            
            start_time = time.time()
            buffer = bytearray()
            
            while time.time() - start_time < test_duration:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    buffer.extend(data)
                    total_bytes += len(data)
                    
                    # Шукати CRSF пакети
                    while len(buffer) >= 4:
                        if buffer[0] == 0xC8:  # CRSF sync
                            packet_len = buffer[1] + 2
                            if packet_len <= len(buffer) and packet_len <= 64:
                                # Можливий CRSF пакет
                                packet = buffer[:packet_len]
                                buffer = buffer[packet_len:]
                                
                                # Перевірити CRC
                                if validate_crsf_crc(packet):
                                    crsf_packets += 1
                                    if packet[2] == 0x16:  # RC_CHANNELS_PACKED
                                        print(f"    ✅ CRSF RC packet found!")
                            else:
                                buffer = buffer[1:]
                        else:
                            buffer = buffer[1:]
                
                time.sleep(0.01)
            
            ser.close()
            
            if crsf_packets > 0:
                print(f"  🎯 Found {crsf_packets} CRSF packets at {baud} baud")
                return baud, crsf_packets
            elif total_bytes > 0:
                print(f"  📊 {total_bytes} bytes received, but no valid CRSF")
            else:
                print(f"  ❌ No data at {baud} baud")
                
        except Exception as e:
            print(f"  ❌ Error at {baud}: {e}")
    
    return None, 0

def validate_crsf_crc(packet):
    """Перевірити CRC8 CRSF пакета"""
    if len(packet) < 4:
        return False
    
    crc = 0
    for byte in packet[2:-1]:
        crc = crc ^ byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc = crc << 1
        crc = crc & 0xFF
    
    return crc == packet[-1]

def interactive_port_assignment():
    """Інтерактивне призначення портів"""
    ports = detect_usb_ports()
    
    if len(ports) < 2:
        print(f"❌ Found only {len(ports)} USB port(s), need 2!")
        print(f"Available: {ports}")
        return None, None
    
    print(f"📋 USB-UART Port Detection Results:")
    print("=" * 50)
    
    # Тестувати кожен порт
    port_results = {}
    for port in ports:
        print(f"\n🔌 Testing {port}:")
        baud, packets = test_port_for_crsf(port)
        
        if packets > 0:
            port_results[port] = {'type': 'RX_LIKELY', 'baud': baud, 'packets': packets}
            print(f"  🎮 Likely RX receiver (found CRSF packets)")
        else:
            port_results[port] = {'type': 'FC_LIKELY', 'baud': None, 'packets': 0}
            print(f"  🚁 Likely Flight Controller (no CRSF input)")
    
    # Автоматичне призначення
    rx_port = None
    fc_port = None
    
    for port, info in port_results.items():
        if info['type'] == 'RX_LIKELY' and rx_port is None:
            rx_port = port
        elif info['type'] == 'FC_LIKELY' and fc_port is None:
            fc_port = port
    
    # Fallback призначення
    if rx_port is None:
        rx_port = ports[0]
    if fc_port is None:
        fc_port = ports[1] if len(ports) > 1 else ports[0]
    
    print(f"\n🎯 RECOMMENDED ASSIGNMENT:")
    print(f"  RX Receiver: {rx_port}")
    print(f"  FC: {fc_port}")
    
    # Підтвердження користувачем
    print(f"\n❓ Is this assignment correct?")
    confirm = input("Press Enter to confirm, or 's' to swap: ").strip().lower()
    
    if confirm == 's':
        rx_port, fc_port = fc_port, rx_port
        print(f"🔄 Swapped: RX={rx_port}, FC={fc_port}")
    
    return rx_port, fc_port

def quick_bridge_test(rx_port, fc_port):
    """Швидкий тест bridge"""
    print(f"\n🧪 QUICK BRIDGE TEST")
    print(f"RX: {rx_port} → FC: {fc_port}")
    
    try:
        # Відкрити обидва порти
        rx_ser = serial.Serial(rx_port, 420000, timeout=0.1)
        fc_ser = serial.Serial(fc_port, 420000, timeout=0.1)
        
        print(f"✅ Both ports opened successfully")
        
        # Тест пересилання на 10 секунд
        start_time = time.time()
        packets_forwarded = 0
        
        while time.time() - start_time < 10:
            # RX → FC
            if rx_ser.in_waiting > 0:
                data = rx_ser.read(rx_ser.in_waiting)
                fc_ser.write(data)
                packets_forwarded += len(data)
            
            # FC → RX (телеметрія)
            if fc_ser.in_waiting > 0:
                data = fc_ser.read(fc_ser.in_waiting)
                rx_ser.write(data)
            
            time.sleep(0.001)
        
        rx_ser.close()
        fc_ser.close()
        
        print(f"✅ Test completed: {packets_forwarded} bytes forwarded")
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False

def main():
    print("🔧 USB-UART CRSF BRIDGE DETECTOR")
    print("=" * 40)
    print("Automatically detects RX and FC ports")
    
    # Знайти порти
    ports = detect_usb_ports()
    print(f"\n📋 Found {len(ports)} USB-UART adapter(s):")
    for port in ports:
        info = get_port_info(port)
        print(f"  {port}: {info}")
    
    if len(ports) < 2:
        print(f"\n❌ Need 2 USB-UART adapters, found {len(ports)}")
        print(f"Please connect both RX receiver and FC via USB-UART adapters")
        return
    
    # Інтерактивне призначення
    rx_port, fc_port = interactive_port_assignment()
    
    if rx_port and fc_port:
        # Швидкий тест
        test_ok = quick_bridge_test(rx_port, fc_port)
        
        if test_ok:
            print(f"\n✅ Ready to run CRSF bridge!")
            print(f"Use these settings:")
            print(f"  RX port: {rx_port}")
            print(f"  FC port: {fc_port}")
            print(f"  Baud: 420000 (with 115200 fallback)")
            
            # Запропонувати запуск
            run_bridge = input(f"\n❓ Start CRSF bridge now? (y/n): ").strip().lower()
            if run_bridge == 'y':
                import subprocess
                subprocess.run([
                    "python3", "crsf_bridge.py", 
                    "--rx-port", rx_port,
                    "--fc-port", fc_port
                ])
        else:
            print(f"\n❌ Bridge test failed, check connections")
    else:
        print(f"\n❌ Could not determine port assignment")

if __name__ == "__main__":
    main()