#!/usr/bin/env python3
import socket
import threading
import time
import json
from datetime import datetime
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gst, GObject, GLib

# Ініціалізація GStreamer
Gst.init(None)

class CRSFTelemetryReceiver:
    def __init__(self):
        self.telemetry_data = {
            'gps': {'lat': 0, 'lon': 0, 'speed': 0, 'altitude': 0, 'sats': 0, 'fix': 0},
            'battery': {'voltage': 0, 'current': 0, 'remaining': 0, 'capacity': 0},
            'link': {'uplink_rssi': -100, 'uplink_lq': 0, 'downlink_rssi': -100, 'downlink_lq': 0},
            'attitude': {'roll': 0, 'pitch': 0, 'yaw': 0},
            'flight_mode': 'UNKNOWN',
            'armed': False,
            'last_update': time.time()
        }
        self.lock = threading.Lock()

    def parse_crsf_packet(self, data):
        if len(data) < 4:
            return None

        if data[0] != 0xC8:
            return None

        length = data[1] 
        packet_type = data[2]
        payload = data[3:3+length-2]

        expected_crc = data[3+length-2]
        calculated_crc = self.crsf_crc8(data[2:3+length-2])
        if expected_crc != calculated_crc:
            print(f"[WARN] CRC mismatch: expected {expected_crc:02X}, got {calculated_crc:02X}")
            return None

        with self.lock:
            self.telemetry_data['last_update'] = time.time()

            if packet_type == 0x02:
                if len(payload) >= 15:
                    self.telemetry_data['gps'] = {
                        'lat': int.from_bytes(payload[0:4], 'big', signed=True) / 1e7,
                        'lon': int.from_bytes(payload[4:8], 'big', signed=True) / 1e7,
                        'speed': int.from_bytes(payload[8:10], 'big') / 100,
                        'altitude': int.from_bytes(payload[12:14], 'big') + 1000,
                        'sats': payload[14],
                        'fix': 1 if payload[14] >= 4 else 0
                    }

            elif packet_type == 0x08:
                if len(payload) >= 8:
                    voltage = int.from_bytes(payload[0:2], 'big') / 100
                    current = int.from_bytes(payload[2:4], 'big') / 100
                    capacity = int.from_bytes(payload[4:7], 'big') if len(payload) >= 7 else 0
                    remaining = payload[7] if len(payload) >= 8 else 0

                    self.telemetry_data['battery'] = {
                        'voltage': voltage,
                        'current': current,
                        'capacity': capacity,
                        'remaining': remaining
                    }

            elif packet_type == 0x14:
                if len(payload) >= 10:
                    uplink_rssi = payload[0] - 256 if payload[0] > 127 else payload[0]
                    uplink_lq = payload[1]
                    uplink_snr = payload[2] - 256 if payload[2] > 127 else payload[2] 
                    downlink_rssi = payload[3] - 256 if payload[3] > 127 else payload[3]
                    downlink_lq = payload[4]
                    downlink_snr = payload[5] - 256 if payload[5] > 127 else payload[5]

                    self.telemetry_data['link'] = {
                        'uplink_rssi': uplink_rssi,
                        'uplink_lq': uplink_lq,
                        'uplink_snr': uplink_snr,
                        'downlink_rssi': downlink_rssi, 
                        'downlink_lq': downlink_lq,
                        'downlink_snr': downlink_snr
                    }

            elif packet_type == 0x1E:
                if len(payload) >= 6:
                    roll = int.from_bytes(payload[0:2], 'big', signed=True) / 10000
                    pitch = int.from_bytes(payload[2:4], 'big', signed=True) / 10000  
                    yaw = int.from_bytes(payload[4:6], 'big', signed=True) / 10000

                    self.telemetry_data['attitude'] = {
                        'roll': roll,
                        'pitch': pitch, 
                        'yaw': yaw
                    }

            elif packet_type == 0x21:
                if len(payload) >= 1:
                    mode_byte = payload[0]
                    armed = bool(mode_byte & 0x01)

                    mode_map = {
                        0: 'DISARMED', 1: 'MANUAL', 2: 'ACRO', 3: 'ANGLE',
                        4: 'HORIZON', 5: 'BARO', 6: 'MAG', 7: 'HEADFREE',
                        8: 'HEADADJ', 9: 'CAMSTAB', 10: 'PASSTHRU', 11: 'RANGEFINDER',
                        12: 'FAILSAFE', 13: 'GPS_RESCUE', 14: 'ANTI_GRAVITY'
                    }

                    mode_id = (mode_byte >> 1) & 0x0F
                    flight_mode = mode_map.get(mode_id, f'MODE_{mode_id}')

                    self.telemetry_data['flight_mode'] = flight_mode
                    self.telemetry_data['armed'] = armed

        return True

    def crsf_crc8(self, data):
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0xD5) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def udp_listener(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", 6970))
        print("[INFO] Listening for CRSF telemetry on port 6970")

        while True:
            try:
                data, addr = sock.recvfrom(64)
                if self.parse_crsf_packet(data):
                    print(f"[INFO] Telemetry from {addr[0]}:{addr[1]} ({len(data)} bytes)")
            except Exception as e:
                print(f"[ERROR] UDP error: {e}")

    def get_telemetry_text(self):
        with self.lock:
            gps = self.telemetry_data['gps']
            bat = self.telemetry_data['battery'] 
            link = self.telemetry_data['link']
            att = self.telemetry_data['attitude']

            time_since_update = time.time() - self.telemetry_data['last_update']
            status = "ONLINE" if time_since_update < 2.0 else "OFFLINE"
            gps_status = "GPS FIX" if gps['fix'] else "NO FIX"
            arm_status = "ARMED" if self.telemetry_data['armed'] else "DISARMED"

            return (
                f"=== SPEEDYBEE FC TELEMETRY === {status}\n"
                f"Mode: {self.telemetry_data['flight_mode']} | {arm_status}\n"
                f"GPS: {gps['lat']:.6f}, {gps['lon']:.6f} | {gps_status}\n"
                f"Alt: {gps['altitude']}m | Speed: {gps['speed']:.1f}m/s | Sats: {gps['sats']}\n"
                f"Attitude: R:{att['roll']:.1f}° P:{att['pitch']:.1f}° Y:{att['yaw']:.1f}°\n"
                f"Battery: {bat['voltage']:.1f}V | {bat['current']:.1f}A | {bat['remaining']}%\n"
                f"Link: UL {link['uplink_rssi']}dBm LQ{link['uplink_lq']}% | DL {link['downlink_rssi']}dBm LQ{link['downlink_lq']}%\n"
                f"Time: {datetime.now().strftime('%H:%M:%S')}"
            )

class VideoStreamer:
    def __init__(self, telemetry_receiver):
        self.telemetry = telemetry_receiver
        self.pipeline = None
        self.textoverlay = None

    def create_pipeline(self):
        pipeline_str = """
        udpsrc port=5600 caps="application/x-rtp,encoding-name=H264" !
        rtph264depay !
        h264parse !
        avdec_h264 !
        videoconvert !
        textoverlay name=telemetry_overlay
            text="Waiting for telemetry..." 
            valignment=top 
            halignment=left
            font-desc="Monospace Bold 14"
            color=0xFFFFFFFF
            outline-color=0x000000FF
            line-alignment=left !
        videoconvert !
        autovideosink
        """

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.textoverlay = self.pipeline.get_by_name("telemetry_overlay")

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_message)

        return True

    def on_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            print("[INFO] End-of-stream")
            self.stop()
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"[ERROR] Video error: {err}: {debug}")
            self.stop()

    def update_overlay(self):
        if self.textoverlay:
            telemetry_text = self.telemetry.get_telemetry_text()
            self.textoverlay.set_property("text", telemetry_text)
        return True

    def start(self):
        if not self.create_pipeline():
            return False

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("[ERROR] Failed to start video pipeline")
            return False

        print("[INFO] Video pipeline started")
        GLib.timeout_add(100, self.update_overlay)

        return True

    def stop(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)

class GroundStation:
    def __init__(self):
        self.telemetry_receiver = CRSFTelemetryReceiver()
        self.video_streamer = VideoStreamer(self.telemetry_receiver)

    def send_stop_command(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(b"STOP", ("192.168.0.10", 6969))
            print("[INFO] STOP command sent to camera")
        except Exception as e:
            print(f"[ERROR] Failed to send STOP: {e}")
        finally:
            sock.close()

    def interactive_console(self):
        print("[INFO] Type 'stop' to send STOP command to camera")
        while True:
            cmd = input(">>> ").strip().lower()
            if cmd == "stop":
                self.send_stop_command()
            elif cmd in ("exit", "quit"):
                print("Exiting interactive console...")
                break

    def start(self):
        print("[INFO] CRSF Ground Station starting...")
        telemetry_thread = threading.Thread(target=self.telemetry_receiver.udp_listener, daemon=True)
        telemetry_thread.start()

        if not self.video_streamer.start():
            print("[ERROR] Failed to start video")
            return

        print("[INFO] Video stream active")
        print("[INFO] Telemetry receiver active")
        print("[INFO] Press Ctrl+C to stop or type 'stop'")

        console_thread = threading.Thread(target=self.interactive_console, daemon=True)
        console_thread.start()

        try:
            loop = GLib.MainLoop()
            loop.run()
        except KeyboardInterrupt:
            print("[INFO] Shutting down...")
        finally:
            self.video_streamer.stop()
            print("[INFO] Ground Station stopped")

if __name__ == "__main__":
    ground_station = GroundStation()
    ground_station.start()



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
