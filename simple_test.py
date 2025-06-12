#!/usr/bin/env python3
"""
Аналізатор CRSF - перевіряє що саме відправляється і що приймається
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

def build_reference_crsf_packet(roll=1500, pitch=1500, throttle=1000, yaw=1500):
    """Створити еталонний CRSF пакет як у справжніх передавачах"""
    
    # Конвертувати в CRSF значення (172-1811)
    def to_crsf(val):
        return max(172, min(1811, int(172 + (val - 1000) * (1811 - 172) / 1000)))
    
    channels = [
        to_crsf(roll),     # Ch1 - Roll
        to_crsf(pitch),    # Ch2 - Pitch  
        to_crsf(throttle), # Ch3 - Throttle
        to_crsf(yaw),      # Ch4 - Yaw
        to_crsf(1500),     # Ch5-16 центр
        to_crsf(1500), to_crsf(1500), to_crsf(1500),
        to_crsf(1500), to_crsf(1500), to_crsf(1500), to_crsf(1500),
        to_crsf(1500), to_crsf(1500), to_crsf(1500), to_crsf(1500)
    ]
    
    # Упаковка як у справжньому CRSF
    packed = bytearray(22)
    
    # Метод з OpenTX/EdgeTX
    bit_index = 0
    for channel in channels:
        channel &= 0x7FF  # 11 біт
        
        # Записати канал біт за бітом
        for bit in range(11):
            if channel & (1 << bit):
                byte_idx = bit_index // 8
                bit_pos = bit_index % 8
                if byte_idx < 22:
                    packed[byte_idx] |= (1 << bit_pos)
            bit_index += 1
    
    # Створити фрейм
    frame = bytearray([0xC8, 24, 0x16])  # Sync, Length, Type
    frame.extend(packed)
    
    # CRC8
    crc_table = generate_crc8_table()
    crc = calculate_crc8(frame[2:], crc_table)
    frame.append(crc)
    
    return bytes(frame), channels

def analyze_packet(packet_data):
    """Аналізувати CRSF пакет"""
    if len(packet_data) < 26:
        return f"❌ Packet too short: {len(packet_data)} bytes"
    
    sync = packet_data[0]
    length = packet_data[1] 
    type_byte = packet_data[2]
    payload = packet_data[3:25]
    crc = packet_data[25]
    
    # Перевірити CRC
    crc_table = generate_crc8_table()
    expected_crc = calculate_crc8(packet_data[2:25], crc_table)
    
    result = []
    result.append(f"📦 Packet Analysis:")
    result.append(f"   Sync: 0x{sync:02X} {'✅' if sync == 0xC8 else '❌'}")
    result.append(f"   Length: {length} {'✅' if length == 24 else '❌'}")
    result.append(f"   Type: 0x{type_byte:02X} {'✅' if type_byte == 0x16 else '❌'}")
    result.append(f"   CRC: 0x{crc:02X} vs 0x{expected_crc:02X} {'✅' if crc == expected_crc else '❌'}")
    
    # Декодувати канали
    channels = []
    bit_index = 0
    
    for ch in range(16):
        channel = 0
        for bit in range(11):
            if bit_index < len(payload) * 8:
                byte_idx = bit_index // 8
                bit_pos = bit_index % 8
                if byte_idx < len(payload) and (payload[byte_idx] & (1 << bit_pos)):
                    channel |= (1 << bit)
            bit_index += 1
        channels.append(channel)
    
    result.append(f"📡 Channels:")
    result.append(f"   Roll: {channels[0]:4d}, Pitch: {channels[1]:4d}")
    result.append(f"   Throttle: {channels[2]:4d}, Yaw: {channels[3]:4d}")
    
    return "\n".join(result)

def main():
    port = "/dev/ttyUSB0"
    
    print("🔬 CRSF Protocol Analyzer")
    print("=" * 40)
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.01)
        print(f"✅ UART opened: {port}")
        
        test_cases = [
            ("CENTER", 1500, 1500, 1000, 1500),
            ("ROLL_MIN", 1000, 1500, 1000, 1500), 
            ("ROLL_MAX", 2000, 1500, 1000, 1500),
            ("PITCH_MIN", 1500, 1000, 1000, 1500),
            ("PITCH_MAX", 1500, 2000, 1000, 1500),
        ]
        
        for test_name, roll, pitch, throttle, yaw in test_cases:
            print(f"\n🧪 Test: {test_name}")
            print(f"   Input: Roll={roll}, Pitch={pitch}, Throttle={throttle}, Yaw={yaw}")
            
            # Створити пакет
            packet, channels = build_reference_crsf_packet(roll, pitch, throttle, yaw)
            
            print(f"   CRSF values: Roll={channels[0]}, Pitch={channels[1]}, Throttle={channels[2]}, Yaw={channels[3]}")
            print(f"   Packet: {packet.hex()}")
            
            # Аналізувати що ми створили
            analysis = analyze_packet(packet)
            print(f"   {analysis.replace(chr(10), chr(10)+'   ')}")
            
            # Відправити пакет 100 разів
            print(f"   📡 Sending {test_name} packets...")
            for _ in range(100):
                ser.write(packet)
                time.sleep(0.02)  # 50 Hz
            
            input(f"   ⏸️  Check Betaflight Receiver tab for {test_name}, then press Enter...")
        
        print(f"\n✅ Analysis complete!")
        
    except KeyboardInterrupt:
        print(f"\n⏹️  Stopped")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    main()