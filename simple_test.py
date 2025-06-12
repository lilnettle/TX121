#!/usr/bin/env python3
"""
ВИПРАВЛЕНИЙ CRSF протокол з правильним Device Address
"""

import serial
import time

# CRSF константи з Betaflight коду
CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_SYNC_BYTE = 0xC8

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

def build_correct_crsf_packet(channels, crc_table):
    """ПРАВИЛЬНИЙ CRSF пакет згідно Betaflight коду"""
    
    # Конвертувати канали в CRSF діапазон (172-1811)
    crsf_channels = []
    for ch in channels:
        if ch < 1000:
            crsf_val = 172
        elif ch > 2000:
            crsf_val = 1811
        else:
            # Лінійна конверсія: 1000->172, 2000->1811
            crsf_val = int(172 + (ch - 1000) * (1811 - 172) / 1000)
        crsf_channels.append(crsf_val)
    
    # Упаковка каналів як в struct crsfPayloadRcChannelsPacked_s
    # 16 каналів по 11 біт = 176 біт = 22 байти
    payload = bytearray(22)
    
    # Упаковка бітів (little-endian order)
    bit_offset = 0
    for channel in crsf_channels[:16]:
        channel &= 0x7FF  # Обмежити до 11 біт
        
        # Записати 11 біт каналу в little-endian порядку
        for bit_idx in range(11):
            if channel & (1 << bit_idx):
                byte_idx = bit_offset // 8
                bit_pos = bit_offset % 8
                if byte_idx < 22:
                    payload[byte_idx] |= (1 << bit_pos)
            bit_offset += 1
    
    # Створити повний фрейм згідно Betaflight структури
    frame = bytearray()
    frame.append(CRSF_ADDRESS_FLIGHT_CONTROLLER)  # Device Address (0xC8)
    frame.append(24)                              # Frame Length (payload + type + crc)
    frame.append(CRSF_FRAMETYPE_RC_CHANNELS_PACKED)  # Type (0x16)
    frame.extend(payload)                         # 22 байти каналів
    
    # CRC від type до кінця payload (як в crsfFrameCRC())
    crc_data = frame[2:]  # Від type включно
    crc = calculate_crc8(crc_data, crc_table)
    frame.append(crc)
    
    return bytes(frame), crsf_channels

def main():
    port = "/dev/ttyUSB0"
    
    print("✅ CORRECT CRSF Test")
    print("=" * 30)
    print("Using proper Betaflight CRSF format")
    print("Device Address: 0xC8 (CRSF_ADDRESS_FLIGHT_CONTROLLER)")
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.01)
        print(f"✅ UART opened: {port}")
        
        crc_table = generate_crc8_table()
        cycle = 0
        
        while True:
            # Базові канали
            channels = [1500] * 16
            channels[2] = 1000  # Throttle мінімум
            
            cycle += 1
            time_sec = cycle / 50.0
            
            # Цикл тестів кожні 3 секунди
            test_phase = int(time_sec) % 12
            
            if test_phase < 3:
                # Тест 1: Roll MIN
                channels[0] = 1000
                channels[1] = 1500  
                channels[3] = 1500
                state = "ROLL_MIN"
            elif test_phase < 6:
                # Тест 2: Roll MAX
                channels[0] = 2000
                channels[1] = 1500
                channels[3] = 1500  
                state = "ROLL_MAX"
            elif test_phase < 9:
                # Тест 3: Pitch MIN/MAX
                channels[0] = 1500
                channels[1] = 1000 if test_phase < 7.5 else 2000
                channels[3] = 1500
                state = "PITCH_MIN" if test_phase < 7.5 else "PITCH_MAX"
            else:
                # Тест 4: ALL CENTER
                channels[0] = 1500
                channels[1] = 1500
                channels[3] = 1500
                state = "CENTER  "
            
            # Створити правильний пакет
            packet, crsf_vals = build_correct_crsf_packet(channels, crc_table)
            ser.write(packet)
            
            # Показати статус
            if cycle % 25 == 0:
                print(f"\r🎮 {state} | "
                      f"Roll:{channels[0]:4d}→{crsf_vals[0]:4d} "
                      f"Pitch:{channels[1]:4d}→{crsf_vals[1]:4d} "
                      f"Yaw:{channels[3]:4d}→{crsf_vals[3]:4d} | "
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