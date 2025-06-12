#!/usr/bin/env python3
"""
Фінальний CRSF тест після правильного налаштування через CLI
"""

import serial
import time
import sys

def generate_crc8_table():
    """Генерація CRC8 таблиці DVB-S2"""
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return table

def calculate_crc8(data, table):
    """Розрахунок CRC8 DVB-S2"""
    crc = 0
    for byte in data:
        crc = table[crc ^ byte]
    return crc

def create_final_crsf_packet(roll=1500, pitch=1500, throttle=1000, yaw=1500):
    """Створити фінальний правильний CRSF пакет"""
    
    # Конвертувати в CRSF діапазон (172-1811)
    def to_crsf(val):
        if val < 1000:
            return 172
        elif val > 2000:
            return 1811
        else:
            return int(172 + (val - 1000) * (1811 - 172) / 1000)
    
    channels = [
        to_crsf(roll), to_crsf(pitch), to_crsf(throttle), to_crsf(yaw),
        to_crsf(1500), to_crsf(1500), to_crsf(1500), to_crsf(1500),
        to_crsf(1500), to_crsf(1500), to_crsf(1500), to_crsf(1500),
        to_crsf(1500), to_crsf(1500), to_crsf(1500), to_crsf(1500)
    ]
    
    # Упаковка каналів (176 біт = 22 байти)
    payload = bytearray(22)
    bit_offset = 0
    
    for channel in channels:
        channel &= 0x7FF  # 11 біт
        
        # Записати біти в little-endian порядку (як в Betaflight)
        for bit_idx in range(11):
            if channel & (1 << bit_idx):
                byte_idx = bit_offset // 8
                bit_pos = bit_offset % 8
                if byte_idx < 22:
                    payload[byte_idx] |= (1 << bit_pos)
            bit_offset += 1
    
    # Створити фрейм
    frame = bytearray()
    frame.append(0xC8)  # CRSF_ADDRESS_FLIGHT_CONTROLLER
    frame.append(24)    # Frame Length (22 + 1 type + 1 crc)
    frame.append(0x16)  # CRSF_FRAMETYPE_RC_CHANNELS_PACKED
    frame.extend(payload)
    
    # CRC8
    crc_table = generate_crc8_table()
    crc = calculate_crc8(frame[2:], crc_table)
    frame.append(crc)
    
    return bytes(frame), channels

def check_cli_setup():
    """Перевірити що CLI налаштування виконані"""
    print("🔧 CLI SETUP VERIFICATION")
    print("=" * 40)
    print("Before running this test, make sure you executed in Betaflight CLI:")
    print()
    print("📋 Required CLI commands:")
    print("   set serialrx_provider = CRSF")
    print("   serial 1 64 115200 57600 0 420000")
    print("   save")
    print()
    print("🔍 Then verify with:")
    print("   get serialrx_provider")
    print("   serial")
    print()
    
    response = input("❓ Have you executed these CLI commands? (y/n): ")
    if response.lower() != 'y':
        print("❌ Please run CLI commands first!")
        sys.exit(1)

def test_crsf_final():
    """Фінальний тест CRSF"""
    
    # Спробувати обидва можливі порти
    ports_to_try = [
        ("/dev/ttyUSB0", "USB0 - might be UART0"),
        ("/dev/ttyUSB1", "USB1 - might be UART1"),
        ("/dev/ttyACM0", "ACM0 - alternative USB"),
    ]
    
    for port, description in ports_to_try:
        print(f"\n🧪 Testing {port} ({description})")
        print("-" * 40)
        
        try:
            # Спробувати 420000 baud (CRSF швидкість)
            ser = serial.Serial(port, 420000, timeout=0.01)
            print(f"✅ Opened {port} at 420000 baud")
            
            print("📡 Sending CRSF test sequence...")
            
            # Тестова послідовність
            test_sequence = [
                (1000, 1500, 1000, 1500, "Roll MIN"),
                (1500, 1500, 1000, 1500, "Center"),  
                (2000, 1500, 1000, 1500, "Roll MAX"),
                (1500, 1500, 1000, 1500, "Center"),
                (1500, 1000, 1000, 1500, "Pitch MIN"),
                (1500, 1500, 1000, 1500, "Center"),
                (1500, 2000, 1000, 1500, "Pitch MAX"),
                (1500, 1500, 1000, 1500, "Center"),
            ]
            
            for roll, pitch, throttle, yaw, name in test_sequence:
                packet, channels = create_final_crsf_packet(roll, pitch, throttle, yaw)
                
                print(f"  🎮 {name:10} | Roll:{channels[0]:4d} Pitch:{channels[1]:4d}")
                
                # Відправити пакет 100 разів (2 секунди)
                for _ in range(100):
                    ser.write(packet)
                    time.sleep(0.02)
                
                time.sleep(0.5)  # Пауза між тестами
            
            ser.close()
            
            response = input(f"  ❓ Did you see channels moving in Receiver tab? (y/n): ")
            if response.lower() == 'y':
                print(f"  🎉 SUCCESS! CRSF working on {port}")
                return True, port
            else:
                print(f"  ❌ No movement on {port}")
                
        except Exception as e:
            print(f"  ❌ Failed to test {port}: {e}")
    
    return False, None

def main():
    print("🚀 FINAL CRSF TEST")
    print("=" * 50)
    print("This test assumes you've configured Betaflight CLI properly")
    
    # Перевірити CLI налаштування
    check_cli_setup()
    
    # Запустити фінальний тест
    success, working_port = test_crsf_final()
    
    if success:
        print(f"\n🎉 CRSF IS WORKING!")
        print(f"   Working port: {working_port}")
        print(f"   Baud rate: 420000")
        print(f"   Protocol: CRSF")
        print(f"\n✅ You can now use this setup for your FPV simulator!")
        
        # Запропонувати безперервний тест
        cont = input(f"\n❓ Run continuous test? (y/n): ")
        if cont.lower() == 'y':
            run_continuous_test(working_port)
    else:
        print(f"\n❌ CRSF still not working")
        print(f"\n🔍 Additional troubleshooting:")
        print(f"   1. Double-check CLI commands")
        print(f"   2. Verify physical UART1 connection")
        print(f"   3. Try different FC UART ports")
        print(f"   4. Check FC pinout diagram")

def run_continuous_test(port):
    """Безперервний тест для перевірки"""
    print(f"\n🎮 CONTINUOUS CRSF TEST on {port}")
    print("Press Ctrl+C to stop")
    print("-" * 30)
    
    try:
        ser = serial.Serial(port, 420000, timeout=0.01)
        cycle = 0
        
        while True:
            cycle += 1
            
            # Цикл кожні 3 секунди
            phase = (cycle // 150) % 4
            
            if phase == 0:
                packet, ch = create_final_crsf_packet(1000, 1500, 1000, 1500)
                state = "ROLL_MIN "
            elif phase == 1:
                packet, ch = create_final_crsf_packet(2000, 1500, 1000, 1500)
                state = "ROLL_MAX "
            elif phase == 2:
                packet, ch = create_final_crsf_packet(1500, 1000, 1000, 1500)
                state = "PITCH_MIN"
            else:
                packet, ch = create_final_crsf_packet(1500, 2000, 1000, 1500)
                state = "PITCH_MAX"
            
            ser.write(packet)
            
            if cycle % 25 == 0:
                print(f"\r🎮 {state} | R:{ch[0]:4d} P:{ch[1]:4d} T:{ch[2]:4d} Y:{ch[3]:4d} | "
                      f"Cycle:{cycle:4d}", end="", flush=True)
            
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        print(f"\n⏹️  Continuous test stopped")
        ser.close()

if __name__ == "__main__":
    main()