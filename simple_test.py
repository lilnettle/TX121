#!/usr/bin/env python3
"""
Виправлений CRSF протокол - правильне кодування каналів
"""

import serial
import time

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
    """Розрахунок CRC8"""
    crc = 0
    for byte in data:
        crc = table[crc ^ byte]
    return crc

def build_crsf_packet_fixed(channels, crc_table):
    """Виправлене створення CRSF пакету"""
    
    # Конвертувати канали в правильний діапазон CRSF
    # CRSF: 172-1811 (11 біт, але зсунуто)
    crsf_channels = []
    for ch in channels:
        # Конвертувати з 1000-2000 в 172-1811
        if ch < 1000:
            crsf_val = 172
        elif ch > 2000:
            crsf_val = 1811
        else:
            # Лінійна інтерполяція: 1000->172, 2000->1811
            crsf_val = int(172 + (ch - 1000) * (1811 - 172) / 1000)
        crsf_channels.append(crsf_val)
    
    # Упакувати 16 каналів по 11 біт кожен
    payload = bytearray(22)  # 16*11=176 біт = 22 байти
    
    bit_offset = 0
    for i, channel in enumerate(crsf_channels[:16]):
        # Обмежити до 11 біт
        channel &= 0x7FF
        
        # Записати 11-бітний канал у payload
        byte_offset = bit_offset // 8
        bit_in_byte = bit_offset % 8
        
        # Записати біти по частинах
        remaining_bits = 11
        while remaining_bits > 0 and byte_offset < 22:
            bits_to_write = min(8 - bit_in_byte, remaining_bits)
            mask = (1 << bits_to_write) - 1
            
            # Витягти біти з каналу
            shift = remaining_bits - bits_to_write
            bits = (channel >> shift) & mask
            
            # Записати в payload
            payload[byte_offset] |= bits << (8 - bit_in_byte - bits_to_write)
            
            remaining_bits -= bits_to_write
            bit_in_byte = 0
            byte_offset += 1
        
        bit_offset += 11
    
    # Створити повний фрейм
    frame = bytearray()
    frame.append(0xC8)      # Sync byte
    frame.append(24)        # Length (22 payload + 1 type + 1 crc)
    frame.append(0x16)      # Type (RC channels)
    frame.extend(payload)   # Channel data
    
    # Розрахувати CRC
    crc_data = frame[2:]    # Від type до кінця payload
    crc = calculate_crc8(crc_data, crc_table)
    frame.append(crc)
    
    return bytes(frame)

def main():
    port = "/dev/ttyUSB0"
    
    print("🔧 Fixed CRSF Test")
    print("=" * 30)
    print("Testing corrected channel encoding")
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.01)
        print(f"✅ UART opened: {port}")
        
        crc_table = generate_crc8_table()
        cycle = 0
        
        while True:
            # Базові канали в нормальному діапазоні (1000-2000)
            channels = [1500] * 16  # Центр
            channels[2] = 1000      # Throttle мінімум
            
            cycle += 1
            
            # Тест різних значень
            time_sec = cycle / 50.0  # 50 Hz
            
            if int(time_sec) % 4 == 0:
                # Тест 1: MIN значення
                channels[0] = 1000  # Roll MIN
                channels[1] = 1000  # Pitch MIN  
                channels[3] = 1000  # Yaw MIN
                state = "MIN "
            elif int(time_sec) % 4 == 1:
                # Тест 2: MAX значення
                channels[0] = 2000  # Roll MAX
                channels[1] = 2000  # Pitch MAX
                channels[3] = 2000  # Yaw MAX
                state = "MAX "
            elif int(time_sec) % 4 == 2:
                # Тест 3: Різні значення
                channels[0] = 1200  # Roll
                channels[1] = 1800  # Pitch
                channels[3] = 1300  # Yaw
                state = "MIX "
            else:
                # Тест 4: Центр
                channels[0] = 1500  # Roll центр
                channels[1] = 1500  # Pitch центр
                channels[3] = 1500  # Yaw центр
                state = "CTR "
            
            # Створити і відправити виправлений пакет
            packet = build_crsf_packet_fixed(channels, crc_table)
            ser.write(packet)
            
            # Показати стан
            if cycle % 25 == 0:
                print(f"\r🎮 {state} Roll:{channels[0]:4d} Pitch:{channels[1]:4d} "
                      f"Yaw:{channels[3]:4d} Throttle:{channels[2]:4d} "
                      f"Time:{time_sec:5.1f}s", end="", flush=True)
            
            time.sleep(0.02)  # 50 Hz
            
    except KeyboardInterrupt:
        print(f"\n⏹️  Stopped")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    main()