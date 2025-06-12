#!/usr/bin/env python3
"""
RADXA Optimized TX12 контролер з правильним UART налаштуванням
Адаптовано для RADXA Rock 3A/5B з множинними UART портами
"""

import serial
import time
import struct
import signal
import sys
import threading
import atexit
import os
import glob
from typing import List, Optional, Dict

try:
    from evdev import InputDevice, categorize, ecodes, list_devices
    EVDEV_AVAILABLE = True
except ImportError:
    print("❌ ERROR: evdev library not found!")
    print("Install with: sudo apt update && sudo apt install python3-evdev")
    print("Or: pip3 install evdev")
    sys.exit(1)

# RADXA specific UART configurations
RADXA_UART_PORTS = [
    # Основні UART порти на RADXA
    "/dev/ttyS1",    # UART1 (рекомендований для польотного контролера)
    "/dev/ttyS2",    # UART2  
    "/dev/ttyS3",    # UART3
    "/dev/ttyS4",    # UART4
    "/dev/ttyAML0",  # Додатковий UART (деякі моделі)
    "/dev/ttyACM0",  # USB-Serial (якщо через USB)
    "/dev/ttyUSB0",  # USB-Serial адаптер
    "/dev/ttyUSB1",  # Резервний USB-Serial
]

# TX12 специфічна конфігурація (без змін)
TX12_DEVICE_INFO = {
    'name_patterns': ['OpenTX RadioMaster TX12', 'radiomaster', 'tx12'],
    'vendor_id': None,
    'product_id': None,
}

# TX12 Axis Mapping (залишаємо оригінальний)
TX12_AXIS_MAPPING = {
    ecodes.ABS_X: {
        'channel': 0, 'name': 'Roll', 'type': 'stick',
        'invert': False, 'center': 1062, 'deadzone': (1052, 1072), 'range': (10, 2047)
    },
    ecodes.ABS_Y: {
        'channel': 1, 'name': 'Pitch', 'type': 'stick', 
        'invert': True, 'center': 1036, 'deadzone': (1026, 1046), 'range': (28, 2047)
    },
    ecodes.ABS_Z: {
        'channel': 2, 'name': 'Throttle', 'type': 'throttle',
        'invert': False, 'center': 2, 'deadzone': (0, 10), 'range': (0, 2024)
    },
    ecodes.ABS_RX: {
        'channel': 3, 'name': 'Yaw', 'type': 'stick',
        'invert': False, 'center': 1290, 'deadzone': (1280, 1300), 'range': (184, 2047)
    },
    ecodes.ABS_RY: {
        'channel': 4, 'name': 'AUX1', 'type': 'aux',
        'invert': False, 'center': 1023, 'deadzone': (1013, 1033), 'range': (0, 2047)
    },
    ecodes.ABS_THROTTLE: {
        'channel': 5, 'name': 'AUX2', 'type': 'aux',
        'invert': False, 'center': 1023, 'deadzone': (1013, 1033), 'range': (0, 2047)
    },
    ecodes.ABS_RZ: {
        'channel': 6, 'name': 'AUX3', 'type': 'aux',
        'invert': False, 'center': 1023, 'deadzone': (1013, 1033), 'range': (0, 2047)
    },
}

# TX12 Button Mapping (залишаємо оригінальний)
TX12_BUTTON_MAPPING = {
    ecodes.BTN_A: {'channel': 5, 'name': 'Button_A', 'function': 'arm_toggle'},
    ecodes.BTN_B: {'channel': 6, 'name': 'Button_B', 'function': 'mode_switch'},
    ecodes.BTN_C: {'channel': 7, 'name': 'Button_C', 'function': 'aux3'},
    ecodes.BTN_X: {'channel': 8, 'name': 'Button_X', 'function': 'aux4'},
    ecodes.BTN_Y: {'channel': 9, 'name': 'Button_Y', 'function': 'aux5'},
    ecodes.BTN_Z: {'channel': 10, 'name': 'Button_Z', 'function': 'aux6'},
    ecodes.BTN_TL: {'channel': 11, 'name': 'Left_Bumper', 'function': 'aux7'},
    ecodes.BTN_TR: {'channel': 12, 'name': 'Right_Bumper', 'function': 'aux8'},
    ecodes.BTN_TL2: {'channel': 13, 'name': 'Left_Trigger_Btn', 'function': 'aux9'},
    ecodes.BTN_TR2: {'channel': 14, 'name': 'Right_Trigger_Btn', 'function': 'aux10'},
    ecodes.BTN_SELECT: {'channel': None, 'name': 'Select', 'function': 'menu'},
    ecodes.BTN_START: {'channel': None, 'name': 'Start', 'function': 'calibrate'},
    ecodes.BTN_MODE: {'channel': None, 'name': 'Mode', 'function': 'profile_switch'},
    ecodes.BTN_THUMBL: {'channel': 15, 'name': 'Left_Stick_Press', 'function': 'aux11'},
    ecodes.BTN_THUMBR: {'channel': 16, 'name': 'Right_Stick_Press', 'function': 'aux12'},
}

# CRSF/ELRS константи
CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_CENTER = 992
CRSF_CHANNEL_MAX = 1811

# RADXA UART налаштування
DEFAULT_UART_BAUDRATES = [
    115200,  # Стандартний для CRSF/ELRS
    57600,   # Класичний для польотних контролерів
    9600,    # Повільний режим
    230400,  # Високошвидкісний
    460800,  # Дуже швидкий
]

SEND_RATE = 150  # Hz (150Hz стандартний для ELRS)

class RADXATx12Controller:
    """RADXA оптимізований TX12 контролер"""
    
    def __init__(self, uart_port: Optional[str] = None, baudrate: int = 115200):
        self.uart = None
        self.tx12_device = None
        self.running = False
        
        # UART налаштування
        self.uart_port = uart_port
        self.uart_baudrate = baudrate
        self.detected_uart_port = None
        
        # 16 каналів CRSF (0-15)
        self.channels = [CRSF_CHANNEL_CENTER] * 16
        self.channels[2] = CRSF_CHANNEL_MIN  # Throttle на мінімум для безпеки
        
        # Стан кнопок та контролів
        self.button_states = {}
        self.last_input_time = time.time()
        
        # Статистика
        self.packets_sent = 0
        self.start_time = time.time()
        self.connection_quality = 100.0
        
        # Throttle налаштування
        self.throttle_axis = ecodes.ABS_THROTTLE
        self.throttle_config = TX12_AXIS_MAPPING[self.throttle_axis]
        
        # Реєстрація обробників виходу
        atexit.register(self.cleanup_on_exit)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        print("🎮 RADXA TX12 Controller initialized")
        self.print_radxa_info()
    
    def print_radxa_info(self):
        """Показати інформацію про RADXA конфігурацію"""
        print(f"\n🏗️  RADXA Configuration:")
        print(f"   Platform: RADXA Rock Series")
        print(f"   UART Ports: {len(RADXA_UART_PORTS)} available")
        print(f"   Baudrates: {DEFAULT_UART_BAUDRATES}")
        print(f"   Protocol: CRSF/ELRS")
        print(f"   Send Rate: {SEND_RATE} Hz")
        
        # Перевірити GPIO стан (якщо доступний)
        self.check_radxa_gpio_status()
    
    def check_radxa_gpio_status(self):
        """Перевірити стан GPIO та UART на RADXA"""
        try:
            # Перевірити чи GPIO налаштовані для UART
            gpio_paths = [
                "/sys/class/gpio/",
                "/sys/kernel/debug/gpio",
                "/proc/device-tree/",
            ]
            
            uart_info = []
            for port in RADXA_UART_PORTS:
                if os.path.exists(port):
                    try:
                        # Перевірити права доступу
                        access = os.access(port, os.R_OK | os.W_OK)
                        uart_info.append(f"   {port}: {'✅ Available' if access else '❌ No permission'}")
                    except:
                        uart_info.append(f"   {port}: ❓ Unknown")
                else:
                    uart_info.append(f"   {port}: ❌ Not found")
            
            if uart_info:
                print(f"\n📡 UART Status:")
                for info in uart_info[:6]:  # Показати перші 6
                    print(info)
                    
        except Exception as e:
            print(f"⚠️  GPIO check failed: {e}")
    
    def signal_handler(self, signum, frame):
        """Обробник сигналів"""
        print(f"\n🛑 Received signal {signum}, stopping RADXA controller...")
        self.cleanup_on_exit()
        sys.exit(0)
    
    def cleanup_on_exit(self):
        """Правильне закриття всіх ресурсів"""
        if hasattr(self, 'running'):
            self.running = False
        
        # Закрити TX12
        if hasattr(self, 'tx12_device') and self.tx12_device:
            try:
                self.tx12_device.close()
                print("✅ TX12 device closed")
            except:
                pass
            self.tx12_device = None
        
        # Закрити UART з правильним очищенням буferів
        if hasattr(self, 'uart') and self.uart and self.uart.is_open:
            try:
                # Відправити мінімальні значення для безпеки
                self.send_failsafe_packet()
                
                # Очистити буфери
                self.uart.flush()
                time.sleep(0.05)
                self.uart.reset_output_buffer()
                self.uart.reset_input_buffer()
                time.sleep(0.05)
                
                self.uart.close()
                print("✅ UART closed properly")
            except:
                pass
            self.uart = None
        
        # Дати час системі для звільнення порту
        time.sleep(0.2)
        print("🏁 RADXA cleanup completed")
    
    def send_failsafe_packet(self):
        """Відправити безпечний пакет при закритті"""
        try:
            # Безпечні значення: центр для стіків, мінімум для throttle
            safe_channels = [CRSF_CHANNEL_CENTER] * 16
            safe_channels[2] = CRSF_CHANNEL_MIN  # Throttle
            
            temp_channels = self.channels
            self.channels = safe_channels
            packet = self.build_crsf_packet()
            
            if self.uart and self.uart.is_open:
                self.uart.write(packet)
                self.uart.flush()
            
            self.channels = temp_channels
            print("📡 Failsafe packet sent")
        except:
            pass
    
    def find_available_uart_port(self) -> Optional[str]:
        """Знайти доступний UART порт на RADXA"""
        print("🔍 Scanning RADXA UART ports...")
        
        available_ports = []
        
        for port in RADXA_UART_PORTS:
            try:
                if not os.path.exists(port):
                    continue
                
                # Перевірити права доступу
                if not os.access(port, os.R_OK | os.W_OK):
                    print(f"  ❌ {port}: No permission (try: sudo chmod 666 {port})")
                    continue
                
                # Спробувати відкрити порт
                test_serial = serial.Serial(
                    port=port,
                    baudrate=self.uart_baudrate,
                    timeout=0.1,
                    exclusive=False  # Для тестування
                )
                test_serial.close()
                
                available_ports.append(port)
                print(f"  ✅ {port}: Available")
                
            except serial.SerialException as e:
                if "Device or resource busy" in str(e):
                    print(f"  ⚠️  {port}: Busy")
                else:
                    print(f"  ❌ {port}: {e}")
            except Exception as e:
                print(f"  ❓ {port}: {e}")
        
        if not available_ports:
            print("\n💡 RADXA UART Setup Tips:")
            print("   1. Enable UART in device tree overlay")
            print("   2. Check pinmux configuration")
            print("   3. Set correct permissions: sudo chmod 666 /dev/ttyS*")
            print("   4. Disable console on UART if enabled")
            return None
        
        # Віддати перевагу ttyS1 (рекомендований)
        preferred_order = ["/dev/ttyS1", "/dev/ttyS2", "/dev/ttyACM0", "/dev/ttyUSB0"]
        
        for preferred in preferred_order:
            if preferred in available_ports:
                print(f"✅ Selected: {preferred} (preferred for RADXA)")
                return preferred
        
        # Повернути перший доступний
        selected = available_ports[0]
        print(f"✅ Selected: {selected}")
        return selected
    
    def setup_uart(self) -> bool:
        """Налаштування UART для RADXA"""
        # Автоматично знайти порт якщо не вказаний
        if not self.uart_port:
            self.uart_port = self.find_available_uart_port()
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
                write_timeout=0.05,  # Дати більше часу для запису
                xonxoff=False,       # Вимкнути flow control
                rtscts=False,        # Вимкнути RTS/CTS
                dsrdtr=False,        # Вимкнути DSR/DTR
                exclusive=True       # Ексклюзивний доступ
            )
            
            # Очистити буфери
            self.uart.reset_input_buffer()
            self.uart.reset_output_buffer()
            
            # Тестова відправка
            test_packet = self.build_crsf_packet()
            self.uart.write(test_packet)
            self.uart.flush()
            
            print(f"✅ UART ready: {self.uart_port}")
            self.detected_uart_port = self.uart_port
            return True
            
        except serial.SerialException as e:
            print(f"❌ UART setup failed: {e}")
            if "Permission denied" in str(e):
                print(f"💡 Fix: sudo chmod 666 {self.uart_port}")
            elif "Device or resource busy" in str(e):
                print("💡 Fix: Stop other programs using the port")
            return False
        except Exception as e:
            print(f"❌ UART setup failed: {e}")
            return False
    
    def find_tx12(self) -> Optional[str]:
        """Знайти TX12 контролер (без змін)"""
        print("🔍 Searching for TX12 (OpenTX RadioMaster)...")
        
        for path in list_devices():
            try:
                device = InputDevice(path)
                name_lower = device.name.lower()
                
                print(f"  📱 {device.name}")
                
                if any(pattern.lower() in name_lower for pattern in TX12_DEVICE_INFO['name_patterns']):
                    print(f"✅ TX12 found by name: {device.path}")
                    return device.path
                
                capabilities = device.capabilities()
                if ecodes.EV_ABS in capabilities:
                    axes = capabilities[ecodes.EV_ABS]
                    tx12_axes = [ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_THROTTLE]
                    
                    if all(axis in axes for axis in tx12_axes):
                        print(f"✅ TX12 found by axes signature: {device.path}")
                        return device.path
                        
            except Exception as e:
                print(f"  ❌ Error checking {path}: {e}")
        
        print("❌ TX12 not found!")
        return None
    
    def setup_tx12(self) -> bool:
        """Налаштування TX12 (без змін логіки)"""
        device_path = self.find_tx12()
        if not device_path:
            return False
        
        try:
            self.tx12_device = InputDevice(device_path)
            
            TX12_DEVICE_INFO['vendor_id'] = f"0x{self.tx12_device.info.vendor:04x}"
            TX12_DEVICE_INFO['product_id'] = f"0x{self.tx12_device.info.product:04x}"
            
            print(f"✅ TX12 opened: {self.tx12_device.name}")
            print(f"   Vendor/Product: {TX12_DEVICE_INFO['vendor_id']}/{TX12_DEVICE_INFO['product_id']}")
            
            self.auto_detect_throttle()
            return True
        except Exception as e:
            print(f"❌ TX12 setup failed: {e}")
            return False
    
    def auto_detect_throttle(self):
        """Автоматично визначити throttle (без змін)"""
        print(f"🔍 Auto-detecting best throttle axis...")
        
        capabilities = self.tx12_device.capabilities()
        if ecodes.EV_ABS not in capabilities:
            return
        
        available_axes = capabilities[ecodes.EV_ABS]
        throttle_candidates = [
            (ecodes.ABS_THROTTLE, "Dedicated throttle"),
            (ecodes.ABS_Z, "Left trigger"),
            (ecodes.ABS_RY, "Left stick Y"),
        ]
        
        for axis_code, description in throttle_candidates:
            if axis_code in available_axes:
                self.throttle_axis = axis_code
                self.throttle_config = TX12_AXIS_MAPPING[axis_code]
                print(f"✅ Selected throttle: {description} ({ecodes.ABS[axis_code]})")
                break
    
    def convert_axis_to_crsf(self, raw_value: int, axis_config: dict) -> int:
        """Конвертація значень (без змін)"""
        axis_type = axis_config['type']
        axis_range = axis_config['range']
        center = axis_config['center']
        deadzone = axis_config['deadzone']
        invert = axis_config['invert']
        
        if deadzone[0] <= raw_value <= deadzone[1]:
            if axis_type == 'throttle':
                normalized = 0.0
            else:
                normalized = 0.5
        elif raw_value < deadzone[0]:
            normalized = (raw_value - axis_range[0]) / (deadzone[0] - axis_range[0]) * 0.5
        else:
            normalized = (raw_value - deadzone[1]) / (axis_range[1] - deadzone[1]) * 0.5 + 0.5
        
        if axis_type == 'throttle':
            normalized = (raw_value - axis_range[0]) / (axis_range[1] - axis_range[0])
            if axis_config.get('throttle_invert', False):
                normalized = 1.0 - normalized
        
        if invert:
            normalized = 1.0 - normalized
        
        normalized = max(0.0, min(1.0, normalized))
        crsf_value = int(round(CRSF_CHANNEL_MIN + normalized * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)))
        return max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, crsf_value))
    
    def process_tx12_event(self, event):
        """Обробка TX12 подій (без змін)"""
        self.last_input_time = time.time()
        
        if event.type == ecodes.EV_ABS:
            if event.code in TX12_AXIS_MAPPING:
                axis_config = TX12_AXIS_MAPPING[event.code]
                channel_idx = axis_config['channel']
                
                if channel_idx is not None:
                    old_value = self.channels[channel_idx]
                    new_value = self.convert_axis_to_crsf(event.value, axis_config)
                    self.channels[channel_idx] = new_value
                    
                    if axis_config['type'] in ['stick', 'throttle'] and abs(old_value - new_value) > 10:
                        percent = ((new_value - CRSF_CHANNEL_MIN) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) * 100
                        print(f"🕹️  {axis_config['name']:12} → {new_value:4d} ({percent:5.1f}%) "
                              f"[raw: {event.value:6d}]")
        
        elif event.type == ecodes.EV_KEY:
            if event.code in TX12_BUTTON_MAPPING:
                button_config = TX12_BUTTON_MAPPING[event.code]
                button_pressed = bool(event.value)
                self.button_states[event.code] = button_pressed
                
                if button_config['function'] == 'menu' and button_pressed:
                    self.print_current_channels()
                elif button_config['function'] == 'calibrate' and button_pressed:
                    self.center_sticks()
                
                if button_config['channel'] is not None:
                    channel_idx = button_config['channel']
                    if channel_idx < 16:
                        self.channels[channel_idx] = CRSF_CHANNEL_MAX if button_pressed else CRSF_CHANNEL_MIN
                        
                        state_str = "PRESSED" if button_pressed else "released"
                        print(f"🔘 {button_config['name']} → Ch{channel_idx} ({state_str})")
    
    def center_sticks(self):
        """Центрувати стіки"""
        for axis_code, config in TX12_AXIS_MAPPING.items():
            if config['type'] == 'stick' and config['channel'] is not None:
                channel_idx = config['channel']
                if channel_idx < 4:
                    if config['name'] != 'Throttle':
                        self.channels[channel_idx] = CRSF_CHANNEL_CENTER
        print("🎯 Main sticks centered")
    
    def check_failsafe(self):
        """Перевірка failsafe"""
        if time.time() - self.last_input_time > 1.0:
            self.channels[0] = CRSF_CHANNEL_CENTER  # Roll
            self.channels[1] = CRSF_CHANNEL_CENTER  # Pitch  
            self.channels[2] = CRSF_CHANNEL_MIN     # Throttle
            self.channels[3] = CRSF_CHANNEL_CENTER  # Yaw
    
    def build_crsf_packet(self) -> bytes:
        """CRSF пакет (без змін)"""
        packed = bytearray(22)
        bit_offset = 0
        
        for channel in self.channels:
            channel = max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, channel))
            byte_offset = bit_offset // 8
            bit_shift = bit_offset % 8
            remaining_bits = 11
            
            while remaining_bits > 0 and byte_offset < 22:
                bits_to_write = min(8 - bit_shift, remaining_bits)
                mask = (1 << bits_to_write) - 1
                packed[byte_offset] |= ((channel >> (11 - remaining_bits)) & mask) << bit_shift
                remaining_bits -= bits_to_write
                bit_shift = 0
                byte_offset += 1
            
            bit_offset += 11
        
        frame = bytearray()
        frame.append(CRSF_ADDRESS_FLIGHT_CONTROLLER)
        frame.append(len(packed) + 2)
        frame.append(CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        frame.extend(packed)
        
        crc = 0
        for byte in frame[2:]:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0xD5) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        frame.append(crc)
        
        return bytes(frame)
    
    def send_channels(self):
        """Відправка каналів з покращеною обробкою для RADXA"""
        try:
            packet = self.build_crsf_packet()
            if self.uart and self.uart.is_open:
                self.uart.write(packet)
                self.packets_sent += 1
                
                # Оновити якість з'єднання
                self.connection_quality = min(100.0, self.connection_quality + 0.1)
                
        except serial.SerialTimeoutException:
            print("⚠️  UART write timeout")
            self.connection_quality = max(0.0, self.connection_quality - 5.0)
        except Exception as e:
            print(f"❌ Send error: {e}")
            self.connection_quality = max(0.0, self.connection_quality - 10.0)
            if self.connection_quality < 50.0:
                self.running = False
    
    def print_current_channels(self):
        """Показати поточні канали"""
        print(f"\n📋 Current Channels (RADXA → Flight Controller):")
        channel_names = ['Roll', 'Pitch', 'Throttle', 'Yaw', 'AUX1', 'AUX2', 'AUX3', 'AUX4']
        
        for i, (name, value) in enumerate(zip(channel_names, self.channels[:8])):
            percent = ((value - CRSF_CHANNEL_MIN) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) * 100
            print(f"   Ch{i+1:2d} {name:8}: {value:4d} ({percent:5.1f}%)")
        
        print(f"   🌐 Connection: {self.connection_quality:.1f}%")
        print(f"   📡 UART: {self.detected_uart_port}")
        
        active_buttons = [TX12_BUTTON_MAPPING[code]['name'] for code, pressed 
                         in self.button_states.items() if pressed]
        if active_buttons:
            print(f"   🔘 Active: {', '.join(active_buttons)}")
    
    def tx12_loop(self):
        """TX12 читання (без змін)"""
        while self.running:
            try:
                if self.tx12_device:
                    events = self.tx12_device.read()
                    for event in events:
                        if event.type in [ecodes.EV_ABS, ecodes.EV_KEY]:
                            self.process_tx12_event(event)
            except BlockingIOError:
                pass
            except Exception as e:
                print(f"❌ TX12 error: {e}")
                break
            
            time.sleep(0.001)
    
    def main_loop(self):
        """Головний цикл RADXA контролера"""
        print(f"🚀 Starting RADXA TX12 → Flight Controller")
        print(f"📡 UART: {self.detected_uart_port} @ {self.uart_baudrate}")
        print(f"📊 Rate: {SEND_RATE} Hz")
        print(f"🎮 Ready for TX12 input!")
        print(f"💡 Press SELECT button to show channels, START to center sticks")
        
        # Запуск TX12 потоку
        tx12_thread = threading.Thread(target=self.tx12_loop, daemon=True)
        tx12_thread.start()
        
        last_send_time = 0
        last_stats_time = 0
        last_heartbeat_time = 0
        send_interval = 1.0 / SEND_RATE
        
        try:
            while self.running:
                current_time = time.time()
                
                # Відправка каналів
                if current_time - last_send_time >= send_interval:
                    self.check_failsafe()
                    self.send_channels()
                    last_send_time = current_time
                
                # Статистика кожні 5 секунд
                if current_time - last_stats_time >= 5.0:
                    runtime = current_time - self.start_time
                    rate = self.packets_sent / runtime if runtime > 0 else 0
                    print(f"📊 RADXA Stats: {self.packets_sent} packets, "
                          f"Rate: {rate:.1f} Hz, Quality: {self.connection_quality:.1f}%")
                    last_stats_time = current_time
                
                # Heartbeat кожні 30 секунд
                if current_time - last_heartbeat_time >= 30.0:
                    print(f"💓 RADXA Heartbeat: System running normally")
                    last_heartbeat_time = current_time
                
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print(f"\n🛑 Stopping RADXA controller...")
        finally:
            self.cleanup_on_exit()
    
    def start(self) -> bool:
        """Запуск RADXA контролера"""
        print("🔧 Initializing RADXA TX12 Controller...")
        
        if not self.setup_uart():
            print("❌ UART setup failed")
            return False
            
        if not self.setup_tx12():
            print("❌ TX12 setup failed")
            return False
        
        self.running = True
        print("✅ RADXA TX12 Controller ready!")
        return True
    
    def stop(self):
        """Зупинка контролера"""
        self.cleanup_on_exit()

# Додаткові утиліти для RADXA
def check_radxa_system():
    """Перевірити системну конфігурацію RADXA"""
    print("🔍 Checking RADXA system configuration...")
    
    # Перевірити модель RADXA
    try:
        with open("/proc/device-tree/model", "r") as f:
            model = f.read().strip()
            print(f"📱 Model: {model}")
    except:
        print("📱 Model: Unknown RADXA")
    
    # Перевірити ядро
    try:
        with open("/proc/version", "r") as f:
            kernel = f.read().split()[2]
            print(f"🐧 Kernel: {kernel}")
    except:
        pass
    
    # Перевірити користувача
    current_user = os.getenv("USER", "unknown")
    print(f"👤 User: {current_user}")
    
    if current_user != "root":
        print("⚠️  Running as non-root. You may need sudo for UART access")
        print("💡 Tip: Add user to dialout group: sudo usermod -a -G dialout $USER")
    
    # Перевірити групи
    try:
        import grp
        user_groups = [g.gr_name for g in grp.getgrall() if current_user in g.gr_mem]
        if "dialout" in user_groups:
            print("✅ User in dialout group")
        else:
            print("❌ User not in dialout group")
    except:
        pass

def setup_radxa_uart_permissions():
    """Налаштувати права доступу для UART на RADXA"""
    print("🔧 Setting up RADXA UART permissions...")
    
    commands = []
    for port in RADXA_UART_PORTS:
        if os.path.exists(port):
            commands.append(f"sudo chmod 666 {port}")
    
    if commands:
        print("📝 Run these commands to fix permissions:")
        for cmd in commands:
            print(f"   {cmd}")
        
        print("\n💡 Or add to /etc/udev/rules.d/99-radxa-uart.rules:")
        print('   SUBSYSTEM=="tty", GROUP="dialout", MODE="0666"')
    else:
        print("❌ No UART ports found")

def print_radxa_pinout():
    """Показати RADXA UART pinout"""
    print("\n📌 RADXA UART Pinout (40-pin header):")
    print("   UART1 (recommended for flight controller):")
    print("     Pin 8  (GPIO14) → UART1 TX → FC RX")
    print("     Pin 10 (GPIO15) → UART1 RX → FC TX") 
    print("     Pin 6  (GND)    → GND      → FC GND")
    print("   ")
    print("   UART2:")
    print("     Pin 27 (GPIO0)  → UART2 TX")
    print("     Pin 28 (GPIO1)  → UART2 RX")
    print("   ")
    print("   Connection to Flight Controller:")
    print("     RADXA TX → FC RX")
    print("     RADXA RX → FC TX")
    print("     RADXA GND → FC GND")
    print("     ⚠️  Do NOT connect VCC unless you know what you're doing!")

def main():
    """Головна функція"""
    print("=" * 70)
    print("🎮 RADXA TX12 → Flight Controller CRSF Bridge")  
    print("   Optimized for RADXA Rock series with multiple UART support")
    print("   UART auto-detection and proper resource management")
    print("=" * 70)
    
    # Системна перевірка
    check_radxa_system()
    
    # Показати pinout
    print_radxa_pinout()
    
    # Перевірити аргументи командного рядка
    import argparse
    parser = argparse.ArgumentParser(description='RADXA TX12 Controller')
    parser.add_argument('--uart', type=str, help='UART port (auto-detect if not specified)')
    parser.add_argument('--baudrate', type=int, default=115200, help='UART baudrate (default: 115200)')
    parser.add_argument('--setup-permissions', action='store_true', help='Show UART permission setup commands')
    parser.add_argument('--list-ports', action='store_true', help='List available UART ports')
    
    args = parser.parse_args()
    
    # Опції утиліт
    if args.setup_permissions:
        setup_radxa_uart_permissions()
        return
    
    if args.list_ports:
        print("\n🔍 Available UART ports:")
        for port in RADXA_UART_PORTS:
            status = "✅ Exists" if os.path.exists(port) else "❌ Not found"
            print(f"   {port}: {status}")
        return
    
    # Створити та запустити контролер
    controller = RADXATx12Controller(
        uart_port=args.uart,
        baudrate=args.baudrate
    )
    
    if controller.start():
        controller.main_loop()
    else:
        print("❌ Failed to start RADXA controller")
        print("\n💡 Troubleshooting:")
        print("   1. Check UART permissions: python3 script.py --setup-permissions")
        print("   2. List available ports: python3 script.py --list-ports")
        print("   3. Try different baudrate: python3 script.py --baudrate 57600")
        print("   4. Check TX12 connection")

if __name__ == "__main__":
    main()