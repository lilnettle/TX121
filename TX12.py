import socket
import time
import threading
from evdev import InputDevice, categorize, ecodes, list_devices

# == Configuration for RadioMaster TX12 ==
UDP_IP = "192.168.0.100"
UDP_PORT = 6969
SEND_RATE_HZ = 66
FAILSAFE_TIMEOUT = 2.0  # seconds

# == CRSF constants ==
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
        
        self.axis_map = {
            ecodes.ABS_X: 0,
            ecodes.ABS_Y: 1,
            ecodes.ABS_Z: 2,
            ecodes.ABS_RZ: 3,
            ecodes.ABS_RX: 4,
            ecodes.ABS_RY: 5,
        }

        self.switch_map = {
            ecodes.BTN_TRIGGER: 6,
            ecodes.BTN_THUMB: 7,
            ecodes.BTN_THUMB2: 8,
            ecodes.BTN_TOP: 9,
            ecodes.BTN_TOP2: 10,
            ecodes.BTN_PINKIE: 11,
            ecodes.BTN_BASE: 12,
            ecodes.BTN_BASE2: 13,
        }

        self.switch_states = {}

        self.expo_settings = {
            0: 0.2,
            1: 0.2,
            2: 0.0,
            3: 0.2,
        }

    def find_tx12_device(self):
        target_names = ["TX12", "RADIOMASTER", "RADIOMASTER TX12", "FRSKY", "TARANIS", "JOYSTICK"]

        for path in list_devices():
            try:
                d = InputDevice(path)
                device_name = d.name.upper()

                if any(name in device_name for name in target_names):
                    print(f"🎮 RadioMaster TX12 found: {d.name} ({path})")
                    caps = d.capabilities()
                    if ecodes.EV_ABS in caps:
                        axes = [ecodes.ABS[ax[0]] for ax in caps[ecodes.EV_ABS]]
                        print(f"   📊 Available axes: {', '.join(axes)}")
                    if ecodes.EV_KEY in caps:
                        buttons = len(caps[ecodes.EV_KEY])
                        print(f"   🔘 Button count: {buttons}")
                    return d
            except (OSError, PermissionError) as e:
                print(f"⚠️  Access error on {path}: {e}")
                continue
        return None

    def apply_expo(self, value, expo):
        if expo == 0:
            return value
        normalized = (value - CRSF_CHANNEL_MID) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID)
        sign = 1 if normalized >= 0 else -1
        abs_val = abs(normalized)
        expo_val = (1 - expo) * abs_val + expo * (abs_val ** 3)
        result = int(expo_val * sign * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID) + CRSF_CHANNEL_MID)
        return max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, result))

    def crsf_crc8(self, data):
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
        return crc

    def scale_axis(self, val, channel_idx, invert=False):
        if invert:
            val = -val
        scaled = int(((val + 32768) / 65535.0) * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) + CRSF_CHANNEL_MIN)
        scaled = max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, scaled))
        if channel_idx in self.expo_settings:
            scaled = self.apply_expo(scaled, self.expo_settings[channel_idx])
        return scaled

    def pack_crsf_channels(self, ch):
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
        payload = self.pack_crsf_channels(self.channels)
        frame = bytearray([CRSF_ADDRESS, CRSF_TYPE_RC_CHANNELS_PACKED, len(payload)])
        frame.extend(payload)
        frame.append(self.crsf_crc8(frame[2:]))
        return frame

    def apply_failsafe(self):
        print("⚠️  TX12 disconnected - Failsafe engaged")
        self.channels[2] = CRSF_CHANNEL_MIN
        for i in [0, 1, 3]:
            self.channels[i] = CRSF_CHANNEL_MID
        for i in range(4, 16):
            self.channels[i] = CRSF_CHANNEL_MIN

    def transmit_loop(self):
        interval = 1.0 / SEND_RATE_HZ
        packet_count = 0
        while self.running:
            start_time = time.time()
            if time.time() - self.last_input > FAILSAFE_TIMEOUT:
                self.apply_failsafe()
            try:
                packet = self.build_crsf_packet()
                self.sock.sendto(packet, (UDP_IP, UDP_PORT))
                packet_count += 1
                if packet_count % (SEND_RATE_HZ * 5) == 0:
                    print(f"📡 Sent {packet_count} packets")
            except Exception as e:
                print(f"❌ Transmission error: {e}")
            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            time.sleep(sleep_time)

    def process_input(self, event):
        self.last_input = time.time()
        if event.type == ecodes.EV_ABS and event.code in self.axis_map:
            ch = self.axis_map[event.code]
            invert = False
            if event.code == ecodes.ABS_Y:
                invert = True
            elif event.code == ecodes.ABS_Z:
                invert = False
            self.channels[ch] = self.scale_axis(event.value, ch, invert)
        elif event.type == ecodes.EV_KEY and event.code in self.switch_map:
            ch = self.switch_map[event.code]
            if event.value == 1:
                self.channels[ch] = CRSF_CHANNEL_MAX
            elif event.value == 0:
                self.channels[ch] = CRSF_CHANNEL_MIN
            else:
                self.channels[ch] = CRSF_CHANNEL_MID

    def print_status(self):
        main_channels = f"A:{self.channels[0]:4d} E:{self.channels[1]:4d} T:{self.channels[2]:4d} R:{self.channels[3]:4d}"
        aux_channels = " ".join(f"AUX{i-3}:{self.channels[i]:4d}" for i in range(4, 8))
        print(f"\r{main_channels} | {aux_channels}", end="", flush=True)

    def calibrate_sticks(self):
        print("🎯 TX12 stick calibration...")
        print("   Move all sticks to max/min and press Enter")
        input("   Ready? Press Enter...")
        print("✅ Calibration complete")

    def run(self):
        print("🚁 RadioMaster TX12 → CRSF UDP Bridge")
        print("=" * 50)
        self.dev = self.find_tx12_device()
        if not self.dev:
            print("❌ RadioMaster TX12 not found!")
            print("💡 Make sure:")
            print("   - TX12 is connected via USB")
            print("   - Joystick mode is enabled in settings")
            print("   - Permissions are correct (sudo or 'input' group)")
            return
        if input("Run calibration? (y/N): ").lower() == 'y':
            self.calibrate_sticks()
        print(f"📡 Transmitting to {UDP_IP}:{UDP_PORT} at {SEND_RATE_HZ}Hz")
        print("📊 Display mode: A=Aileron E=Elevator T=Throttle R=Rudder")
        print("🛑 Press Ctrl+C to stop")
        self.running = True
        tx_thread = threading.Thread(target=self.transmit_loop, daemon=True)
        tx_thread.start()
        try:
            status_counter = 0
            for event in self.dev.read_loop():
                if not self.running:
                    break
                self.process_input(event)
                status_counter += 1
                if status_counter >= 50:
                    self.print_status()
                    status_counter = 0
        except KeyboardInterrupt:
            print("\n🛑 Transmission stopped...")
        except Exception as e:
            print(f"\n❌ Error: {e}")
        finally:
            self.running = False
            self.sock.close()
            print("👋 Shutdown complete")

if __name__ == "__main__":
    controller = RadioMasterTX12Controller()
    controller.run()
