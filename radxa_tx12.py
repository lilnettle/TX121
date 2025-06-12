#!/usr/bin/env python3
"""
Виправлений RADXA TX12 контролер з правильним CRSF протоколом
Основні виправлення:
1. Правильний CRSF header та адресація
2. Виправлений алгоритм упакування 11-бітних каналів
3. Правильний CRC8 DVB-S2
4. Синхронізація з Betaflight
"""

import serial
import time
import struct
import signal
import sys
import threading
import atexit
import os
from typing import List, Optional

try:
    from evdev import InputDevice, categorize, ecodes, list_devices
    EVDEV_AVAILABLE = True
except ImportError:
    print("❌ ERROR: evdev library not found!")
    print("Install with: sudo apt update && sudo apt install python3-evdev")
    sys.exit(1)

# CRSF Protocol Constants - ВИПРАВЛЕНІ ЗНАЧЕННЯ
CRSF_SYNC_BYTE = 0xC8                    # Правильний sync byte
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_FRAME_SIZE_RC_CHANNELS = 22         # Розмір payload для каналів
CRSF_HEADER_SIZE = 2                     # Device address + Frame length

# CRSF Channel Values - ПРАВИЛЬНІ ДІАПАЗОНИ
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_CENTER = 992
CRSF_CHANNEL_MAX = 1811
CRSF_CHANNEL_RANGE = CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN

# RADXA UART порти
RADXA_UART_PORTS = [
    "/dev/ttyUSB0",  # USB-UART адаптери
    "/dev/ttyUSB1", 
    "/dev/ttyACM0",  # Arduino-сумісні
    "/dev/ttyS1",    # Нативні UART
    "/dev/ttyS2",
    "/dev/ttyFIQ0",
]

# TX12 конфігурація (спрощена для тестування)
TX12_AXIS_MAPPING = {
    ecodes.ABS_X: {'channel': 0, 'name': 'Roll', 'invert': False},
    ecodes.ABS_Y: {'channel': 1, 'name': 'Pitch', 'invert': True},
    ecodes.ABS_Z: {'channel': 2, 'name': 'Throttle', 'invert': False},
    ecodes.ABS_RX: {'channel': 3, 'name': 'Yaw', 'invert': False},
}

class CRSFController:
    """Виправлений CRSF контролер"""
    
    def __init__(self, uart_port: Optional[str] = None, baudrate: int = 115200):
        self.uart = None
        self.tx12_device = None
        self.running = False
        
        self.uart_port = uart_port
        self.uart_baudrate = baudrate
        
        # 16 каналів - ВАЖЛИВО: ініціалізувати правильними значеннями
        self.channels = [CRSF_CHANNEL_CENTER] * 16
        self.channels[2] = CRSF_CHANNEL_MIN  # Throttle на мінімум
        
        self.packets_sent = 0
        self.last_input_time = time.time()
        
        # CRC8 таблиця для DVB-S2
        self.crc8_table = self._generate_crc8_table()
        
        atexit.register(self.cleanup)
        signal.signal(signal.SIGINT, self.signal_handler)
        
        print("🎮 CRSF Controller initialized")
    
    def _generate_crc8_table(self):
        """Генерація CRC8 таблиці для DVB-S2 (0xD5 поліном)"""
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
    
    def _calculate_crc8(self, data: bytes) -> int:
        """Правильний розрахунок CRC8 DVB-S2"""
        crc = 0
        for byte in data:
            crc = self.crc8_table[crc ^ byte]
        return crc
    
    def signal_handler(self, signum, frame):
        """Обробник сигналів"""
        print(f"\n🛑 Stopping controller...")
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """Правильне закриття"""
        self.running = False
        
        if self.tx12_device:
            try:
                self.tx12_device.close()
            except:
                pass
        
        if self.uart and self.uart.is_open:
            try:
                # Відправити failsafe
                self.send_failsafe()
                time.sleep(0.1)
                self.uart.close()
                print("✅ UART closed")
            except:
                pass
    
    def send_failsafe(self):
        """Відправити безпечні значення"""
        try:
            safe_channels = [CRSF_CHANNEL_CENTER] * 16
            safe_channels[2] = CRSF_CHANNEL_MIN  # Throttle
            
            temp = self.channels
            self.channels = safe_channels
            packet = self.build_crsf_packet()
            
            if self.uart and self.uart.is_open:
                self.uart.write(packet)
                self.uart.flush()
            
            self.channels = temp
            print("📡 Failsafe sent")
        except:
            pass
    
    def find_uart_port(self) -> Optional[str]:
        """Знайти доступний UART порт"""
        print("🔍 Scanning UART ports...")
        
        for port in RADXA_UART_PORTS:
            try:
                if not os.path.exists(port):
                    continue
                
                test_serial = serial.Serial(
                    port=port,
                    baudrate=self.uart_baudrate,
                    timeout=0.1
                )
                test_serial.close()
                
                print(f"✅ Found: {port}")
                return port
                
            except Exception as e:
                print(f"❌ {port}: {e}")
        
        return None
    
    def setup_uart(self) -> bool:
        """Налаштування UART"""
        if not self.uart_port:
            self.uart_port = self.find_uart_port()
            if not self.uart_port:
                return False
        
        print(f"🔌 Setting up UART: {self.uart_port} @ {self.uart_baudrate}")
        
        try:
            self.uart = serial.Serial(
                port=self.uart_port,
                baudrate=self.uart_baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=0.01,
                write_timeout=0.05,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # Очистити буфери
            self.uart.reset_input_buffer()
            self.uart.reset_output_buffer()
            
            print(f"✅ UART ready")
            return True
            
        except Exception as e:
            print(f"❌ UART setup failed: {e}")
            return False
    
    def find_tx12(self) -> Optional[str]:
        """Знайти TX12"""
        print("🔍 Searching for TX12...")
        
        for path in list_devices():
            try:
                device = InputDevice(path)
                name_lower = device.name.lower()
                
                print(f"  📱 {device.name}")
                
                if any(pattern in name_lower for pattern in ['radiomaster', 'tx12', 'opentx']):
                    print(f"✅ TX12 found: {device.path}")
                    return device.path
                    
            except Exception as e:
                print(f"  ❌ Error: {e}")
        
        print("❌ TX12 not found!")
        return None
    
    def setup_tx12(self) -> bool:
        """Налаштування TX12"""
        device_path = self.find_tx12()
        if not device_path:
            return False
        
        try:
            self.tx12_device = InputDevice(device_path)
            print(f"✅ TX12 opened: {self.tx12_device.name}")
            return True
        except Exception as e:
            print(f"❌ TX12 setup failed: {e}")
            return False
    
    def convert_to_crsf(self, raw_value: int, axis_config: dict) -> int:
        """Конвертація значень в CRSF"""
        # Припустимо діапазон -32768 до 32767 для джойстика
        input_min = -32768
        input_max = 32767
        input_center = 0
        
        # Нормалізація до 0.0-1.0
        if raw_value <= input_center:
            normalized = 0.5 * (raw_value - input_min) / (input_center - input_min)
        else:
            normalized = 0.5 + 0.5 * (raw_value - input_center) / (input_max - input_center)
        
        # Інверсія якщо потрібно
        if axis_config.get('invert', False):
            normalized = 1.0 - normalized
        
        # Обмеження
        normalized = max(0.0, min(1.0, normalized))
        
        # Конвертація в CRSF діапазон
        crsf_value = int(CRSF_CHANNEL_MIN + normalized * CRSF_CHANNEL_RANGE)
        
        return max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, crsf_value))
    
    def process_tx12_event(self, event):
        """Обробка TX12 подій"""
        self.last_input_time = time.time()
        
        if event.type == ecodes.EV_ABS and event.code in TX12_AXIS_MAPPING:
            axis_config = TX12_AXIS_MAPPING[event.code]
            channel_idx = axis_config['channel']
            
            old_value = self.channels[channel_idx]
            new_value = self.convert_to_crsf(event.value, axis_config)
            self.channels[channel_idx] = new_value
            
            # Лог значних змін
            if abs(old_value - new_value) > 10:
                percent = ((new_value - CRSF_CHANNEL_MIN) / CRSF_CHANNEL_RANGE) * 100
                print(f"🕹️  {axis_config['name']:8} → {new_value:4d} ({percent:5.1f}%) [raw: {event.value:6d}]")
    
    def build_crsf_packet(self) -> bytes:
        """Створення правильного CRSF пакету"""
        # Підготовка payload
        payload = bytearray()
        
        # Упакування 16 каналів по 11 біт кожен
        bits_buffer = 0
        bits_count = 0
        
        for channel in self.channels:
            # Обмеження значення каналу
            channel = max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, channel))
            
            # Додати 11 біт до буферу
            bits_buffer |= (channel << bits_count)
            bits_count += 11
            
            # Витягти повні байти
            while bits_count >= 8:
                payload.append(bits_buffer & 0xFF)
                bits_buffer >>= 8
                bits_count -= 8
        
        # Додати останні біти якщо є
        if bits_count > 0:
            payload.append(bits_buffer & 0xFF)
        
        # Обрізати до правильного розміру (22 байти)
        payload = payload[:CRSF_FRAME_SIZE_RC_CHANNELS]
        
        # Побудова фрейму
        frame = bytearray()
        frame.append(CRSF_SYNC_BYTE)                    # Sync byte
        frame.append(CRSF_FRAME_SIZE_RC_CHANNELS + 2)   # Frame length (payload + type + crc)
        frame.append(CRSF_FRAMETYPE_RC_CHANNELS_PACKED) # Frame type
        frame.extend(payload)                           # Payload
        
        # Розрахунок CRC8 (від frame type до кінця payload)
        crc_data = frame[2:]  # Від frame type
        crc = self._calculate_crc8(crc_data)
        frame.append(crc)
        
        return bytes(frame)
    
    def send_channels(self):
        """Відправка каналів"""
        try:
            packet = self.build_crsf_packet()
            if self.uart and self.uart.is_open:
                self.uart.write(packet)
                self.packets_sent += 1
                
        except Exception as e:
            print(f"❌ Send error: {e}")
    
    def print_channels(self):
        """Показати поточні канали"""
        print(f"\n📋 Current Channels:")
        channel_names = ['Roll', 'Pitch', 'Throttle', 'Yaw']
        
        for i, (name, value) in enumerate(zip(channel_names, self.channels[:4])):
            percent = ((value - CRSF_CHANNEL_MIN) / CRSF_CHANNEL_RANGE) * 100
            print(f"   Ch{i+1} {name:8}: {value:4d} ({percent:5.1f}%)")
        
        print(f"   📡 Packets sent: {self.packets_sent}")
    
    def tx12_loop(self):
        """TX12 читання"""
        while self.running:
            try:
                if self.tx12_device:
                    events = self.tx12_device.read()
                    for event in events:
                        if event.type == ecodes.EV_ABS:
                            self.process_tx12_event(event)
            except BlockingIOError:
                pass
            except Exception as e:
                print(f"❌ TX12 error: {e}")
                break
            
            time.sleep(0.001)
    
    def main_loop(self):
        """Головний цикл"""
        print(f"🚀 Starting CRSF → Flight Controller")
        print(f"📡 UART: {self.uart_port} @ {self.uart_baudrate}")
        print(f"📊 Send rate: 50 Hz")
        print(f"🎮 Ready for input!")
        
        # Запуск TX12 потоку
        tx12_thread = threading.Thread(target=self.tx12_loop, daemon=True)
        tx12_thread.start()
        
        last_send_time = 0
        last_stats_time = 0
        send_interval = 1.0 / 50  # 50 Hz
        
        try:
            while self.running:
                current_time = time.time()
                
                # Відправка каналів
                if current_time - last_send_time >= send_interval:
                    self.send_channels()
                    last_send_time = current_time
                
                # Статистика
                if current_time - last_stats_time >= 5.0:
                    self.print_channels()
                    last_stats_time = current_time
                
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print(f"\n🛑 Stopping...")
        finally:
            self.cleanup()
    
    def start(self) -> bool:
        """Запуск"""
        print("🔧 Initializing CRSF Controller...")
        
        if not self.setup_uart():
            return False
            
        if not self.setup_tx12():
            return False
        
        self.running = True
        print("✅ Controller ready!")
        return True

def test_crsf_without_tx12():
    """Тест CRSF без TX12 - для діагностики"""
    print("🧪 Testing CRSF output without TX12...")
    
    controller = CRSFController()
    
    if not controller.setup_uart():
        print("❌ UART setup failed")
        return
    
    print("📡 Sending test CRSF packets...")
    print("   Check Betaflight Configurator Receiver tab")
    
    controller.running = True
    
    try:
        for i in range(1000):  # 20 секунд тестування
            # Тестові значення
            controller.channels[0] = CRSF_CHANNEL_CENTER + int(500 * (i % 100 - 50) / 50)  # Roll
            controller.channels[1] = CRSF_CHANNEL_CENTER  # Pitch
            controller.channels[2] = CRSF_CHANNEL_MIN     # Throttle
            controller.channels[3] = CRSF_CHANNEL_CENTER  # Yaw
            
            controller.send_channels()
            
            if i % 50 == 0:  # Кожну секунду
                print(f"📊 Sent {i+1} packets, Roll: {controller.channels[0]}")
            
            time.sleep(0.02)  # 50 Hz
            
    except KeyboardInterrupt:
        print("\n🛑 Test stopped")
    finally:
        controller.cleanup()

def main():
    """Головна функція"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Fixed CRSF Controller')
    parser.add_argument('--uart', type=str, help='UART port')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baudrate')
    parser.add_argument('--test', action='store_true', help='Test mode without TX12')
    
    args = parser.parse_args()
    
    if args.test:
        test_crsf_without_tx12()
        return
    
    controller = CRSFController(uart_port=args.uart, baudrate=args.baudrate)
    
    if controller.start():
        controller.main_loop()
    else:
        print("❌ Failed to start controller")
        print("\n💡 Try test mode: python3 script.py --test")

if __name__ == "__main__":
    main()