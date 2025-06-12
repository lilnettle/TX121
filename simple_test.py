#!/usr/bin/env python3
"""
Автоматичний тест - канали повинні рухатись самі
"""

import serial
import time
import math

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

def build_crsf_packet(channels, crc_table):
    """Створення CRSF пакету"""
    payload = bytearray()
    bit_buffer = 0
    bit_count = 0
    
    for channel in channels:
        channel = max(172, min(1811, channel))
        bit_buffer |= (channel << bit_count)
        bit_count += 11
        
        while bit_count >= 8:
            payload.append(bit_buffer & 0xFF)
            bit_buffer >>= 8
            bit_count -= 8
    
    if bit_count > 0:
        payload.append(bit_buffer & 0xFF)
    
    payload = payload[:22]
    
    frame = bytearray()
    frame.append(0xC8)  # Sync
    frame.append(24)    # Length
    frame.append(0x16)  # Type
    frame.extend(payload)
    
    crc_data = frame[2:]
    crc = calculate_crc8(crc_data, crc_table)
    frame.append(crc)
    
    return bytes(frame)

def main():
    port = "/dev/ttyUSB0"
    
    print("🌊 Channel Sweep Test")
    print("=" * 30)
    print("Channels should move automatically in Betaflight Receiver tab!")
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.01)
        print(f"✅ UART opened: {port}")
        
        crc_table = generate_crc8_table()
        
        start_time = time.time()
        packets_sent = 0
        
        while True:
            current_time = time.time() - start_time
            
            # Базові канали (центр)
            channels = [992] * 16
            channels[2] = 172  # Throttle мінімум
            
            # Автоматичний рух каналів
            # Roll (Ch1) - синус
            channels[0] = int(992 + 300 * math.sin(current_time))
            
            # Pitch (Ch2) - косинус  
            channels[1] = int(992 + 200 * math.cos(current_time * 1.5))
            
            # Yaw (Ch4) - треугольник
            yaw_phase = (current_time * 0.5) % 2
            if yaw_phase < 1:
                channels[3] = int(992 + 250 * (yaw_phase * 2 - 1))
            else:
                channels[3] = int(992 + 250 * (2 - yaw_phase * 2))
            
            # Aux1 (Ch5) - квадратна хвиля
            if int(current_time) % 2 == 0:
                channels[4] = 1800
            else:
                channels[4] = 200
            
            # Створити і відправити пакет
            packet = build_crsf_packet(channels, crc_table)
            ser.write(packet)
            packets_sent += 1
            
            # Показувати значення кожні 50 пакетів
            if packets_sent % 50 == 0:
                print(f"\r🎮 Roll:{channels[0]:4d} Pitch:{channels[1]:4d} "
                      f"Yaw:{channels[3]:4d} Aux1:{channels[4]:4d} "
                      f"Packets:{packets_sent:4d}", end="", flush=True)
            
            time.sleep(0.02)  # 50 Hz
            
    except KeyboardInterrupt:
        print(f"\n⏹️  Stopped after {packets_sent} packets")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    main()