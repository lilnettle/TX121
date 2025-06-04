import socket
import time
import threading
from evdev import InputDevice, categorize, ecodes, list_devices

# == Конфігурація для RadioMaster TX12 ==
UDP_IP = "192.168.42.1"
UDP_PORT = 6969
SEND_RATE_HZ = 66
FAILSAFE_TIMEOUT = 2.0  # секунди

# == CRSF константи ==
CRSF_ADDRESS = 0xC8
CRSF_TYPE_RC_CHANNELS_PACKED = 0x16
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_MID = 992
CRSF_CHANNEL_MAX = 1811

class RadioMasterTX12Controller:
    def __init__(self):
        self.channels = [CRSF_CHANNEL_MID] * 16
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = False
        self.last_input = time.time()
        self.dev = None
        
        # Специфічне співставлення для RadioMaster TX12
        # Стандартне Mode 2 налаштування (газ зліва)
        self.axis_map = {
            ecodes.ABS_X: 0,      # Aileron (Roll) - правий стік горизонтально
            ecodes.ABS_Y: 1,      # Elevator (Pitch) - правий стік вертикально
            ecodes.ABS_Z: 2,      # Throttle (Газ) - лівий стік вертикально
            ecodes.ABS_RZ: 3,     # Rudder (Yaw) - лівий стік горизонтально
            ecodes.ABS_RX: 4,     # Aux 1 - потенціометр або слайдер
            ecodes.ABS_RY: 5,     # Aux 2 - потенціометр або слайдер
        }
        
        # Перемикачі та кнопки TX12
        self.switch_map = {
            ecodes.BTN_TRIGGER: 6,    # SA перемикач (2-позиційний)
            ecodes.BTN_THUMB: 7,      # SB перемикач (3-позиційний)
            ecodes.BTN_THUMB2: 8,     # SC перемикач (3-позиційний)
            ecodes.BTN_TOP: 9,        # SD перемикач (2-позиційний)
            ecodes.BTN_TOP2: 10,      # SE перемикач
            ecodes.BTN_PINKIE: 11,    # SF перемикач
            ecodes.BTN_BASE: 12,      # SG перемикач
            ecodes.BTN_BASE2: 13,     # SH перемикач
        }
        
        # Стан перемикачів для 3-позиційних
        self.switch_states = {}
        
        # Налаштування кривих та експоненти
        self.expo_settings = {
            0: 0.2,  # Roll expo
            1: 0.2,  # Pitch expo
            2: 0.0,  # Throttle (лінійний)
            3: 0.2,  # Yaw expo
        }
        
    def find_tx12_device(self):
        """Специфічний пошук RadioMaster TX12"""
        target_names = [
            "TX12", "RADIOMASTER", "RADIOMASTER TX12", 
            "FRSKY", "TARANIS", "JOYSTICK"
        ]
        
        for path in list_devices():
            try:
                d = InputDevice(path)
                device_name = d.name.upper()
                
                if any(name in device_name for name in target_names):
                    print(f"🎮 Знайдено RadioMaster TX12: {d.name} ({path})")
                    
                    # Виведення доступних осей та кнопок
                    caps = d.capabilities()
                    if ecodes.EV_ABS in caps:
                        axes = [ecodes.ABS[ax[0]] for ax in caps[ecodes.EV_ABS]]
                        print(f"   📊 Доступні осі: {', '.join(axes)}")
                    
                    if ecodes.EV_KEY in caps:
                        buttons = len(caps[ecodes.EV_KEY])
                        print(f"   🔘 Кількість кнопок: {buttons}")
                    
                    return d
                    
            except (OSError, PermissionError) as e:
                print(f"⚠️  Помилка доступу до {path}: {e}")
                continue
                
        return None
    
    def apply_expo(self, value, expo):
        """Застосування експоненціальної кривої"""
        if expo == 0:
            return value
            
        # Нормалізація до діапазону -1 до 1
        normalized = (value - CRSF_CHANNEL_MID) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID)
        
        # Застосування експоненти
        sign = 1 if normalized >= 0 else -1
        abs_val = abs(normalized)
        expo_val = (1 - expo) * abs_val + expo * (abs_val ** 3)
        
        # Повернення до CRSF діапазону
        result = int(expo_val * sign * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID) + CRSF_CHANNEL_MID)
        return max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, result))
    
    def crsf_crc8(self, data):
        """Розрахунок CRC8 для CRSF"""
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
        return crc
    
    def scale_axis(self, val, channel_idx, invert=False):
        """Масштабування з урахуванням специфіки TX12"""
        if invert:
            val = -val
            
        # Базове масштабування
        scaled = int(((val + 32768) / 65535.0) * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) + CRSF_CHANNEL_MIN)
        scaled = max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, scaled))
        
        # Застосування експоненти для основних осей
        if channel_idx in self.expo_settings:
            scaled = self.apply_expo(scaled, self.expo_settings[channel_idx])
            
        return scaled
    
    def pack_crsf_channels(self, ch):
        """Упакування каналів у CRSF формат"""
        buf = 0
        bits = 0
        packed = bytearray()
        
        for val in ch:
            val = max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, val))
            buf |= (val & 0x7FF) << bits
            bits += 11
            
            while bits >= 8:
                packed.append(buf & 0xFF)
                buf >>= 8
                bits -= 8
                
        return packed[:22]
    
    def build_crsf_packet(self):
        """Побудова CRSF пакету"""
        payload = self.pack_crsf_channels(self.channels)
        frame = bytearray([CRSF_ADDRESS, CRSF_TYPE_RC_CHANNELS_PACKED, len(payload)])
        frame.extend(payload)
        frame.append(self.crsf_crc8(frame[2:]))
        return frame
    
    def apply_failsafe(self):
        """Застосування захисних значень"""
        print("⚠️  Втрата зв'язку з TX12 - активація захисного режиму")
        
        # Газ на мінімум (важливо для безпеки!)
        self.channels[2] = CRSF_CHANNEL_MIN
        
        # Інші канали в нейтральне положення
        for i in [0, 1, 3]:  # Roll, Pitch, Yaw
            self.channels[i] = CRSF_CHANNEL_MID
            
        # Aux канали в безпечні положення
        for i in range(4, 16):
            self.channels[i] = CRSF_CHANNEL_MIN
    
    def transmit_loop(self):
        """Основний цикл передачі"""
        interval = 1.0 / SEND_RATE_HZ
        packet_count = 0
        
        while self.running:
            start_time = time.time()
            
            # Перевірка тайм-ауту
            if time.time() - self.last_input > FAILSAFE_TIMEOUT:
                self.apply_failsafe()
            
            # Передача пакету
            try:
                packet = self.build_crsf_packet()
                self.sock.sendto(packet, (UDP_IP, UDP_PORT))
                packet_count += 1
                
                # Статистика кожні 5 секунд
                if packet_count % (SEND_RATE_HZ * 5) == 0:
                    print(f"📡 Передано {packet_count} пакетів")
                    
            except Exception as e:
                print(f"❌ Помилка передачі: {e}")
            
            # Підтримання частоти
            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            time.sleep(sleep_time)
    
    def process_input(self, event):
        """Обробка вводу з TX12"""
        self.last_input = time.time()
        
        if event.type == ecodes.EV_ABS and event.code in self.axis_map:
            ch = self.axis_map[event.code]
            
            # Специфічні налаштування для TX12
            invert = False
            if event.code == ecodes.ABS_Y:  # Elevator зазвичай інвертований
                invert = True
            elif event.code == ecodes.ABS_Z:  # Throttle може потребувати інверсії
                invert = False  # Залежить від налаштувань TX12
                
            self.channels[ch] = self.scale_axis(event.value, ch, invert)
            
        elif event.type == ecodes.EV_KEY and event.code in self.switch_map:
            ch = self.switch_map[event.code]
            
            # Обробка різних типів перемикачів
            if event.value == 1:  # Натиснуто
                self.channels[ch] = CRSF_CHANNEL_MAX
            elif event.value == 0:  # Відпущено
                self.channels[ch] = CRSF_CHANNEL_MIN
            else:  # Середнє положення для 3-позиційних
                self.channels[ch] = CRSF_CHANNEL_MID
    
    def print_status(self):
        """Відображення стану каналів"""
        # Основні канали керування
        main_channels = f"A:{self.channels[0]:4d} E:{self.channels[1]:4d} T:{self.channels[2]:4d} R:{self.channels[3]:4d}"
        
        # Aux канали
        aux_channels = " ".join(f"AUX{i-3}:{self.channels[i]:4d}" for i in range(4, 8))
        
        print(f"\r{main_channels} | {aux_channels}", end="", flush=True)
    
    def calibrate_sticks(self):
        """Простий процес калібрування"""
        print("🎯 Калібрування стіків TX12...")
        print("   Рухайте всі стіки у крайні положення і натисніть Enter")
        input("   Готово? Натисніть Enter...")
        print("✅ Калібрування завершено")
    
    def run(self):
        """Основний цикл роботи"""
        print("🚁 RadioMaster TX12 → CRSF UDP Bridge")
        print("=" * 50)
        
        self.dev = self.find_tx12_device()
        if not self.dev:
            print("❌ RadioMaster TX12 не знайдено!")
            print("💡 Переконайтеся що:")
            print("   - TX12 підключений через USB")
            print("   - Увімкнений режим джойстика в налаштуваннях")
            print("   - Встановлені необхідні дозволи (sudo або група input)")
            return
        
        # Опціональне калібрування
        if input("Виконати калібрування? (y/N): ").lower() == 'y':
            self.calibrate_sticks()
        
        print(f"📡 Передача на {UDP_IP}:{UDP_PORT} з частотою {SEND_RATE_HZ}Гц")
        print("📊 Режим відображення: A=Aileron E=Elevator T=Throttle R=Rudder")
        print("🛑 Для зупинки натисніть Ctrl+C")
        
        self.running = True
        
        # Запуск потоку передачі
        tx_thread = threading.Thread(target=self.transmit_loop, daemon=True)
        tx_thread.start()
        
        try:
            status_counter = 0
            for event in self.dev.read_loop():
                if not self.running:
                    break
                    
                self.process_input(event)
                
                # Оновлення статусу
                status_counter += 1
                if status_counter >= 50:  # Частіше оновлення для TX12
                    self.print_status()
                    status_counter = 0
                    
        except KeyboardInterrupt:
            print("\n🛑 Зупинка передачі...")
        except Exception as e:
            print(f"\n❌ Помилка: {e}")
        finally:
            self.running = False
            self.sock.close()
            print("👋 Вимкнено")

if __name__ == "__main__":
    controller = RadioMasterTX12Controller()
    controller.run()