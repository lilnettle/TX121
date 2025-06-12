#!/usr/bin/env python3
"""
Ручний тест каналів - ви контролюєте значення з клавіатури
"""

import serial
import time
import threading

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

class ChannelController:
    def __init__(self):
        self.channels = [992] * 16  # Центр
        self.channels[2] = 172      # Throttle мінімум
        self.running = True
        
    def print_controls(self):
        print("\n🎮 Manual Channel Control")
        print("=" * 40)
        print("Controls:")
        print("  w/s - Roll (Ch1)")
        print("  a/d - Pitch (Ch2)")  
        print("  q/e - Throttle (Ch3)")
        print("  z/x - Yaw (Ch4)")
        print("  r   - Reset all to center")
        print("  ESC - Exit")
        print("\nWatch Betaflight Receiver tab!")
        
    def update_channel(self, ch, delta):
        """Оновити канал"""
        self.channels[ch] = max(172, min(1811, self.channels[ch] + delta))
        
    def print_status(self):
        """Показати поточні значення"""
        print(f"\rRoll:{self.channels[0]:4d} Pitch:{self.channels[1]:4d} "
              f"Throttle:{self.channels[2]:4d} Yaw:{self.channels[3]:4d}", end="", flush=True)

def input_thread(controller):
    """Потік для обробки введення"""
    try:
        while controller.running:
            try:
                key = input().strip().lower()
                
                if key == 'w':
                    controller.update_channel(0, 50)  # Roll +
                elif key == 's':
                    controller.update_channel(0, -50) # Roll -
                elif key == 'a':
                    controller.update_channel(1, 50)  # Pitch +
                elif key == 'd':
                    controller.update_channel(1, -50) # Pitch -
                elif key == 'q':
                    controller.update_channel(2, 50)  # Throttle +
                elif key == 'e':
                    controller.update_channel(2, -50) # Throttle -
                elif key == 'z':
                    controller.update_channel(3, 50)  # Yaw +
                elif key == 'x':
                    controller.update_channel(3, -50) # Yaw -
                elif key == 'r':
                    controller.channels = [992] * 16  # Reset
                    controller.channels[2] = 172      # Throttle мінімум
                elif key == '' or key == 'exit':
                    controller.running = False
                    break
                    
            except EOFError:
                controller.running = False
                break
            except:
                pass
                
    except:
        controller.running = False

def main():
    port = "/dev/ttyUSB0"
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.01)
        print(f"✅ UART opened: {port}")
        
        controller = ChannelController()
        controller.print_controls()
        
        # Запустити потік для введення
        input_t = threading.Thread(target=input_thread, args=(controller,))
        input_t.daemon = True
        input_t.start()
        
        crc_table = generate_crc8_table()
        packets_sent = 0
        
        print(f"\n📡 Sending manual CRSF commands...")
        
        while controller.running:
            # Створити і відправити пакет
            packet = build_crsf_packet(controller.channels, crc_table)
            ser.write(packet)
            packets_sent += 1
            
            # Показати статус кожні 50 пакетів
            if packets_sent % 50 == 0:
                controller.print_status()
            
            time.sleep(0.01)  # 50 Hz
            
    except KeyboardInterrupt:
        print(f"\n⏹️  Stopped")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()
            
    print(f"\n📊 Sent {packets_sent} packets")

if __name__ == "__main__":
    main()