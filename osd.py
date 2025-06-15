#!/usr/bin/env python3
"""
GStreamer HDMI вихід з CRSF телеметрією OSD
Виводить відео з накладеним OSD безпосередньо на HDMI
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import serial
import time
import threading
import argparse
import math
from dataclasses import dataclass
from enum import IntEnum

# Ініціалізація GStreamer
Gst.init(None)

# CRSF парсер
CRSF_SYNC = 0xC8

class PacketsTypes(IntEnum):
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    BARO_ALT = 0x09
    HEARTBEAT = 0x0B
    VIDEO_TRANSMITTER = 0x0F
    LINK_STATISTICS = 0x14
    RC_CHANNELS_PACKED = 0x16
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_PING = 0x28
    DEVICE_INFO = 0x29
    PARAMETER_SETTINGS_ENTRY = 0x2B
    PARAMETER_READ = 0x2C
    PARAMETER_WRITE = 0x2D
    RADIO_ID = 0x3A

@dataclass
class TelemetryData:
    """Структура телеметрії для OSD"""
    # Link
    rssi: int = -999
    link_quality: int = 0
    snr: int = 0
    
    # Battery
    voltage: float = 0.0
    current: float = 0.0
    battery_percent: int = 0
    
    # GPS
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    satellites: int = 0
    ground_speed: float = 0.0
    
    # Attitude (в градусах)
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    
    # RC первые 4 канала
    ch1: int = 1500
    ch2: int = 1500
    ch3: int = 1500
    ch4: int = 1500
    
    # Flight mode
    flight_mode: str = "UNKNOWN"
    
    # Timestamps
    last_update: float = 0.0
    link_last_update: float = 0.0
    battery_last_update: float = 0.0
    gps_last_update: float = 0.0

class CRSFParser:
    """CRSF парсер"""
    
    @staticmethod
    def crc8_dvb_s2(crc, a) -> int:
        crc = crc ^ a
        for ii in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc = crc << 1
        return crc & 0xFF

    @staticmethod
    def crc8_data(data) -> int:
        crc = 0
        for a in data:
            crc = CRSFParser.crc8_dvb_s2(crc, a)
        return crc

    @staticmethod
    def validate_frame(frame) -> bool:
        if len(frame) < 4:
            return False
        return CRSFParser.crc8_data(frame[2:-1]) == frame[-1]

    @staticmethod
    def signed_byte(b):
        return b - 256 if b >= 128 else b

    @staticmethod
    def parse_packet(frame_data, telemetry: TelemetryData):
        """Розпарсити пакет і оновити телеметрію"""
        if len(frame_data) < 3:
            return
        
        ptype = frame_data[2]
        data = frame_data
        current_time = time.time()
        
        try:
            if ptype == PacketsTypes.LINK_STATISTICS and len(data) >= 10:
                telemetry.rssi = CRSFParser.signed_byte(data[3])
                telemetry.link_quality = data[5]
                telemetry.snr = CRSFParser.signed_byte(data[6])
                telemetry.link_last_update = current_time
                
            elif ptype == PacketsTypes.BATTERY_SENSOR and len(data) >= 11:
                telemetry.voltage = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
                telemetry.current = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10.0
                telemetry.battery_percent = data[10]
                telemetry.battery_last_update = current_time
                
            elif ptype == PacketsTypes.GPS and len(data) >= 18:
                telemetry.latitude = int.from_bytes(data[3:7], byteorder='big', signed=True) / 1e7
                telemetry.longitude = int.from_bytes(data[7:11], byteorder='big', signed=True) / 1e7
                telemetry.ground_speed = int.from_bytes(data[11:13], byteorder='big', signed=True) / 36.0
                telemetry.altitude = int.from_bytes(data[15:17], byteorder='big', signed=True) - 1000
                telemetry.satellites = data[17]
                telemetry.gps_last_update = current_time
                
            elif ptype == PacketsTypes.ATTITUDE and len(data) >= 9:
                telemetry.pitch = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10000.0 * 57.29578
                telemetry.roll = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10000.0 * 57.29578
                telemetry.yaw = int.from_bytes(data[7:9], byteorder='big', signed=True) / 10000.0 * 57.29578
                
            elif ptype == PacketsTypes.RC_CHANNELS_PACKED and len(data) >= 24:
                bits = int.from_bytes(data[3:25], 'little')
                telemetry.ch1 = (bits >> (0 * 11)) & 0x7FF
                telemetry.ch2 = (bits >> (1 * 11)) & 0x7FF
                telemetry.ch3 = (bits >> (2 * 11)) & 0x7FF
                telemetry.ch4 = (bits >> (3 * 11)) & 0x7FF
                    
            elif ptype == PacketsTypes.FLIGHT_MODE:
                mode_str = data[3:].decode('utf-8', errors='ignore').rstrip('\x00')
                telemetry.flight_mode = mode_str
                
            telemetry.last_update = current_time
            
        except Exception as e:
            print(f"Parse error: {e}")

class HDMIOSDPlayer:
    """HDMI плеєр з OSD"""
    
    def __init__(self, rtsp_input=None, serial_port="/dev/ttyUSB0", baud_rate=420000, 
                 resolution="1920x1080", framerate=30, fullscreen=True):
        self.rtsp_input = rtsp_input
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.resolution = resolution
        self.framerate = framerate
        self.fullscreen = fullscreen
        
        self.pipeline = None
        self.loop = None
        self.serial_conn = None
        self.running = False
        
        self.telemetry = TelemetryData()
        self.serial_buffer = bytearray()
        
        # GStreamer елементи для динамічного оновлення
        self.text_overlays = {}
        
        # Розпарсити роздільність
        self.width, self.height = map(int, resolution.split('x'))
    
    def connect_serial(self):
        """Підключитися до CRSF"""
        try:
            self.serial_conn = serial.Serial(
                self.serial_port, self.baud_rate,
                timeout=0.01, bytesize=8, parity='N', stopbits=1
            )
            print(f"✅ Connected to CRSF: {self.serial_port} @ {self.baud_rate}")
            return True
        except Exception as e:
            print(f"❌ CRSF connection failed: {e}")
            print("⚠️ Continuing without telemetry")
            return False
    
    def create_gstreamer_pipeline(self):
        """Створити GStreamer pipeline для HDMI виходу"""
        
        # Вибрати джерело відео
        if self.rtsp_input:
            # RTSP вхід
            video_source = f"""
            rtspsrc location={self.rtsp_input} latency=200 drop-on-latency=true ! 
            rtph264depay ! 
            avdec_h264 ! 
            videoscale ! 
            video/x-raw,width={self.width},height={self.height} !
            videoconvert !
            """
        else:
            # Тестовий патерн
            video_source = f"""
            videotestsrc pattern=ball ! 
            video/x-raw,width={self.width},height={self.height},framerate={self.framerate}/1 ! 
            videoconvert !
            """
        
        # Створити pipeline string з OSD overlay
        pipeline_str = video_source
        
        # Додати textoverlay елементи для кожного OSD елемента
        overlays = [
            # RSSI & Link Quality (верх зліва)
            {
                'name': 'rssi_overlay',
                'text': 'RSSI: -- dBm\\nLQ: ---',
                'x': 30, 'y': 40,
                'halignment': 'left', 'valignment': 'top',
                'font': 'Sans Bold 18',
                'color': '0xFF00FF00'
            },
            # Battery (верх справа)
            {
                'name': 'battery_overlay',
                'text': 'BATT: -.--V\\n--%',
                'x': -30, 'y': 40,
                'halignment': 'right', 'valignment': 'top',
                'font': 'Sans Bold 18',
                'color': '0xFF00FF00'
            },
            # GPS (низ зліва)
            {
                'name': 'gps_overlay',
                'text': 'GPS: -- SATs\\nALT: ---m',
                'x': 30, 'y': -40,
                'halignment': 'left', 'valignment': 'bottom',
                'font': 'Sans Bold 16',
                'color': '0xFF00FF00'
            },
            # Attitude (низ справа)
            {
                'name': 'attitude_overlay',
                'text': 'PITCH: --°\\nROLL: --°\\nYAW: --°',
                'x': -30, 'y': -40,
                'halignment': 'right', 'valignment': 'bottom',
                'font': 'Sans Bold 16',
                'color': '0xFF00FF00'
            },
            # Flight Mode (центр зверху)
            {
                'name': 'mode_overlay',
                'text': 'MODE: UNKNOWN',
                'x': 0, 'y': 80,
                'halignment': 'center', 'valignment': 'top',
                'font': 'Sans Bold 20',
                'color': '0xFF00FFFF'
            },
            # RC Channels (центр знизу)
            {
                'name': 'rc_overlay',
                'text': 'CH1-4: ---- ---- ---- ----',
                'x': 0, 'y': -80,
                'halignment': 'center', 'valignment': 'bottom',
                'font': 'Sans Bold 14',
                'color': '0xFFFFFF00'
            }
        ]
        
        # Додати кожен overlay до pipeline
        for overlay in overlays:
            pipeline_str += f"""
            textoverlay name={overlay['name']}
                text="{overlay['text']}"
                halignment={overlay['halignment']} valignment={overlay['valignment']}
                x-absolute={overlay['x']} y-absolute={overlay['y']}
                font-desc="{overlay['font']}"
                color={overlay['color']} !
            """
        
        # Додати crosshair overlay
        pipeline_str += """
        cairooverlay name=crosshair_overlay !
        """
        
        # Вихід на дисплей
        if self.fullscreen:
            # Повноекранний режим
            pipeline_str += f"""
            videoconvert ! 
            videoscale ! 
            video/x-raw,width={self.width},height={self.height} !
            autovideosink sync=false
            """
        else:
            # Віконний режим
            pipeline_str += f"""
            videoconvert ! 
            videoscale ! 
            video/x-raw,width={self.width},height={self.height} !
            ximagesink sync=false force-aspect-ratio=true
            """
        
        print("🎬 GStreamer Pipeline:")
        print(pipeline_str.replace('!', '!\n'))
        
        return pipeline_str
    
    def setup_pipeline(self):
        """Налаштувати GStreamer pipeline"""
        pipeline_str = self.create_gstreamer_pipeline()
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        if not self.pipeline:
            print("❌ Failed to create pipeline")
            return False
        
        # Отримати посилання на overlay елементи
        overlay_names = ['rssi_overlay', 'battery_overlay', 'gps_overlay', 
                        'attitude_overlay', 'mode_overlay', 'rc_overlay']
        
        for name in overlay_names:
            overlay = self.pipeline.get_by_name(name)
            if overlay:
                self.text_overlays[name] = overlay
            else:
                print(f"⚠️ Warning: {name} not found")
        
        # Налаштувати crosshair overlay
        crosshair = self.pipeline.get_by_name('crosshair_overlay')
        if crosshair:
            crosshair.connect('draw', self.draw_crosshair)
        
        # Обробник повідомлень
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message', self.on_message)
        
        return True
    
    def on_message(self, bus, message):
        """Обробник повідомлень GStreamer"""
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"❌ GStreamer Error: {err}")
            print(f"Debug: {debug}")
            self.loop.quit()
        elif message.type == Gst.MessageType.EOS:
            print("📺 End of stream")
            self.loop.quit()
        elif message.type == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old, new, pending = message.parse_state_changed()
                print(f"🎬 Pipeline state: {old.value_nick} -> {new.value_nick}")
    
    def draw_crosshair(self, overlay, cr, timestamp, duration, user_data):
        """Намалювати crosshair і штучний горизонт"""
        center_x = self.width / 2
        center_y = self.height / 2
        crosshair_size = int(self.width * 0.02)  # 2% від ширини екрану
        
        # Налаштування ліній
        cr.set_source_rgba(0, 1, 0, 0.8)  # Зелений
        cr.set_line_width(3)
        
        # Горизонтальна лінія
        cr.move_to(center_x - crosshair_size, center_y)
        cr.line_to(center_x + crosshair_size, center_y)
        cr.stroke()
        
        # Вертикальна лінія
        cr.move_to(center_x, center_y - crosshair_size)
        cr.line_to(center_x, center_y + crosshair_size)
        cr.stroke()
        
        # Центральна точка
        cr.arc(center_x, center_y, 4, 0, 2 * math.pi)
        cr.fill()
        
        # Штучний горизонт
        if abs(self.telemetry.roll) > 1 or abs(self.telemetry.pitch) > 1:
            # Лінія горизонту
            horizon_y = center_y + self.telemetry.pitch * 3  # Масштабування
            roll_rad = self.telemetry.roll * math.pi / 180  # Градуси в радіани
            
            horizon_len = self.width * 0.1  # 10% від ширини
            cos_roll = math.cos(roll_rad)
            sin_roll = math.sin(roll_rad)
            
            x1 = center_x - horizon_len * cos_roll
            y1 = horizon_y + horizon_len * sin_roll
            x2 = center_x + horizon_len * cos_roll
            y2 = horizon_y - horizon_len * sin_roll
            
            cr.set_source_rgba(0, 1, 0, 1)  # Яскравий зелений
            cr.set_line_width(4)
            cr.move_to(x1, y1)
            cr.line_to(x2, y2)
            cr.stroke()
            
            # Маркери крену
            for angle in [-30, -15, 15, 30]:
                angle_rad = angle * math.pi / 180
                marker_len = horizon_len * 0.3
                
                mx1 = center_x - marker_len * math.cos(angle_rad)
                my1 = center_y + marker_len * math.sin(angle_rad)
                mx2 = center_x + marker_len * math.cos(angle_rad)
                my2 = center_y - marker_len * math.sin(angle_rad)
                
                cr.set_line_width(2)
                cr.move_to(mx1, my1)
                cr.line_to(mx2, my2)
                cr.stroke()
    
    def update_osd_text(self):
        """Оновити текст OSD елементів"""
        current_time = time.time()
        
        # RSSI & Link Quality
        if current_time - self.telemetry.link_last_update < 5:
            rssi_color = 0xFF00FF00 if self.telemetry.rssi > -70 else 0xFFFF0000
            rssi_text = f"RSSI: {self.telemetry.rssi}dBm\\nLQ: {self.telemetry.link_quality:03d}"
        else:
            rssi_color = 0xFFFF0000
            rssi_text = "RSSI: NO LINK\\nLQ: ---"
        
        if 'rssi_overlay' in self.text_overlays:
            self.text_overlays['rssi_overlay'].set_property('text', rssi_text)
            self.text_overlays['rssi_overlay'].set_property('color', rssi_color)
        
        # Battery
        if current_time - self.telemetry.battery_last_update < 10:
            batt_color = 0xFF00FF00 if self.telemetry.voltage > 14.0 else 0xFFFF0000
            batt_text = f"BATT: {self.telemetry.voltage:.1f}V"
            if self.telemetry.battery_percent > 0:
                batt_text += f"\\n{self.telemetry.battery_percent}%"
            else:
                batt_text += "\\n--%"
        else:
            batt_color = 0xFFFF0000
            batt_text = "BATT: NO DATA\\n--%"
        
        if 'battery_overlay' in self.text_overlays:
            self.text_overlays['battery_overlay'].set_property('text', batt_text)
            self.text_overlays['battery_overlay'].set_property('color', batt_color)
        
        # GPS
        if current_time - self.telemetry.gps_last_update < 10 and self.telemetry.satellites > 0:
            gps_color = 0xFF00FF00 if self.telemetry.satellites >= 6 else 0xFFFFFF00
            gps_text = f"GPS: {self.telemetry.satellites} SATs\\nALT: {self.telemetry.altitude:.0f}m"
        else:
            gps_color = 0xFFFF0000
            gps_text = "GPS: NO FIX\\nALT: ---m"
        
        if 'gps_overlay' in self.text_overlays:
            self.text_overlays['gps_overlay'].set_property('text', gps_text)
            self.text_overlays['gps_overlay'].set_property('color', gps_color)
        
        # Attitude
        attitude_text = f"PITCH: {self.telemetry.pitch:.1f}°\\nROLL: {self.telemetry.roll:.1f}°\\nYAW: {self.telemetry.yaw:.1f}°"
        if 'attitude_overlay' in self.text_overlays:
            self.text_overlays['attitude_overlay'].set_property('text', attitude_text)
        
        # Flight Mode
        mode_text = f"MODE: {self.telemetry.flight_mode}"
        if 'mode_overlay' in self.text_overlays:
            self.text_overlays['mode_overlay'].set_property('text', mode_text)
        
        # RC Channels
        rc_text = f"CH1-4: {self.telemetry.ch1} {self.telemetry.ch2} {self.telemetry.ch3} {self.telemetry.ch4}"
        if 'rc_overlay' in self.text_overlays:
            self.text_overlays['rc_overlay'].set_property('text', rc_text)
    
    def read_telemetry(self):
        """Читати телеметрію з CRSF"""
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    self.serial_buffer.extend(data)
                    
                    # Обробити фрейми
                    while len(self.serial_buffer) > 2:
                        if self.serial_buffer[0] != CRSF_SYNC:
                            self.serial_buffer.pop(0)
                            continue
                            
                        expected_len = self.serial_buffer[1] + 2
                        if expected_len > 64 or expected_len < 4:
                            self.serial_buffer.pop(0)
                            continue
                            
                        if len(self.serial_buffer) < expected_len:
                            break
                            
                        frame = bytes(self.serial_buffer[:expected_len])
                        self.serial_buffer[:expected_len] = []
                        
                        if CRSFParser.validate_frame(frame):
                            CRSFParser.parse_packet(frame, self.telemetry)
                
                time.sleep(0.001)  # 1ms
                
            except Exception as e:
                print(f"Telemetry read error: {e}")
                time.sleep(0.01)
    
    def update_osd_loop(self):
        """Періодично оновлювати OSD"""
        while self.running:
            self.update_osd_text()
            time.sleep(0.1)  # 10 FPS оновлення OSD
    
    def print_status(self):
        """Вивести статус телеметрії"""
        while self.running:
            current_time = time.time()
            
            # Статус підключення
            link_status = "🟢 CONNECTED" if current_time - self.telemetry.link_last_update < 5 else "🔴 NO LINK"
            batt_status = "🟢 OK" if current_time - self.telemetry.battery_last_update < 10 else "🔴 NO DATA"
            gps_status = f"🟢 {self.telemetry.satellites} SATs" if self.telemetry.satellites > 0 else "🔴 NO FIX"
            
            print(f"\r📊 RSSI: {self.telemetry.rssi}dBm {link_status} | "
                  f"🔋 {self.telemetry.voltage:.1f}V {batt_status} | "
                  f"🛰️ {gps_status} | "
                  f"✈️ {self.telemetry.flight_mode}", end="", flush=True)
            
            time.sleep(2)
    
    def run(self):
        """Запустити HDMI плеєр з OSD"""
        # Підключити CRSF
        self.connect_serial()
        
        # Налаштувати pipeline
        if not self.setup_pipeline():
            return False
        
        self.running = True
        
        # Запустити потоки
        if self.serial_conn:
            telemetry_thread = threading.Thread(target=self.read_telemetry, daemon=True)
            telemetry_thread.start()
            
            status_thread = threading.Thread(target=self.print_status, daemon=True)
            status_thread.start()
        
        osd_thread = threading.Thread(target=self.update_osd_loop, daemon=True)
        osd_thread.start()
        
        # Запустити pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("❌ Failed to start pipeline")
            return False
        
        # Головний loop
        self.loop = GLib.MainLoop()
        
        print("🎬 HDMI OSD Player running!")
        print(f"📺 Resolution: {self.resolution}")
        print(f"📡 Input: {self.rtsp_input or 'Test Pattern'}")
        print("📊 CRSF OSD overlay active")
        print("Press Ctrl+C to stop\n")
        
        try:
            self.loop.run()
        except KeyboardInterrupt:
            print("\n🛑 Stopping...")
        finally:
            self.running = False
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
            if self.serial_conn:
                self.serial_conn.close()
        
        return True

def main():
    parser = argparse.ArgumentParser(description='HDMI Output with CRSF OSD')
    parser.add_argument('-i', '--input', help='RTSP input URL (optional - uses test pattern if not provided)')
    parser.add_argument('-p', '--port', default='/dev/ttyUSB0', help='CRSF serial port')
    parser.add_argument('-b', '--baud', type=int, default=420000, help='CRSF baud rate')
    parser.add_argument('-r', '--resolution', default='1920x1080', 
                       choices=['1920x1080', '1280x720', '3840x2160', '2560x1440'],
                       help='Output resolution')
    parser.add_argument('-f', '--framerate', type=int, default=30, help='Frame rate')
    parser.add_argument('-w', '--windowed', action='store_true', help='Run in windowed mode (not fullscreen)')
    
    args = parser.parse_args()
    
    print("🎬 HDMI OUTPUT WITH CRSF OSD")
    print("=" * 50)
    print(f"Input: {args.input or 'Test Pattern'}")
    print(f"CRSF: {args.port} @ {args.baud}")
    print(f"Output: HDMI {args.resolution} @ {args.framerate}fps")
    print(f"Mode: {'Windowed' if args.windowed else 'Fullscreen'}")
    print()
    print("OSD Features:")
    print("  🎯 Crosshair + Artificial Horizon")
    print("  📊 RSSI & Link Quality")
    print("  🔋 Battery Status")
    print("  🛰️ GPS Information")
    print("  🎮 RC Channels")
    print("  ✈️ Flight Mode & Attitude")
    print()
    
    # Запустити плеєр
    player = HDMIOSDPlayer(
        rtsp_input=args.input,
        serial_port=args.port,
        baud_rate=args.baud,
        resolution=args.resolution,
        framerate=args.framerate,
        fullscreen=not args.windowed
    )
    
    player.run()

if __name__ == "__main__":
    main()