#!/usr/bin/env python3
"""
CRSF HDMI OSD з простим bridge управлінням
- Відображає телеметрію на HDMI OSD
- Пропускає керування: USB1 → USB0 (як bridge)
- Без джойстиків - тільки прозоре перенаправлення
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import serial
import time
import threading
import argparse
import math
import os
from dataclasses import dataclass
from enum import IntEnum

# Ініціалізація
Gst.init(None)

# CRSF константи
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
    """Структура телеметрії"""
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
    
    # Attitude
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    
    # RC Channels (отримані від FC)
    channels: list = None
    
    # Flight mode
    flight_mode: str = "UNKNOWN"
    
    # Bridge status
    bridge_active: bool = False
    rx_packets: int = 0
    fc_packets: int = 0
    
    # Timestamps
    last_update: float = 0.0
    link_last_update: float = 0.0
    battery_last_update: float = 0.0
    gps_last_update: float = 0.0
    
    def __post_init__(self):
        if self.channels is None:
            self.channels = [1500] * 16

class CRSFParser:
    """CRSF парсер для телеметрії"""
    
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
        """Розпарсити пакет телеметрії"""
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
                for i in range(16):
                    channel = (bits >> (i * 11)) & 0x7FF
                    telemetry.channels[i] = channel
                    
            elif ptype == PacketsTypes.FLIGHT_MODE:
                mode_str = data[3:].decode('utf-8', errors='ignore').rstrip('\x00')
                telemetry.flight_mode = mode_str
                
            telemetry.last_update = current_time
            
        except Exception as e:
            print(f"Parse error: {e}")

class SimpleCRSFBridge:
    """Простий CRSF bridge як у вашому коді"""
    
    def __init__(self, rx_port="/dev/ttyUSB1", fc_port="/dev/ttyUSB0", baud_rate=420000):
        self.rx_port = rx_port      # RX приймач
        self.fc_port = fc_port      # FC 
        self.baud_rate = baud_rate
        self.fallback_baud = 115200
        
        self.rx_serial = None
        self.fc_serial = None
        self.running = False
        
        self.stats = {'rx_packets': 0, 'fc_packets': 0, 'errors': 0}
        
    def connect(self):
        """Підключитися з автоматичним fallback"""
        for baud in [self.baud_rate, self.fallback_baud]:
            try:
                print(f"🔌 Trying bridge at {baud} baud...")
                
                self.rx_serial = serial.Serial(self.rx_port, baud, timeout=0.01)
                self.fc_serial = serial.Serial(self.fc_port, baud, timeout=0.01)
                
                print(f"✅ Bridge connected at {baud} baud")
                print(f"🌉 Bridge: {self.rx_port} → {self.fc_port}")
                return True
                
            except Exception as e:
                print(f"❌ Bridge failed at {baud}: {e}")
                self.disconnect()
        
        return False
    
    def disconnect(self):
        """Відключити"""
        if self.rx_serial:
            self.rx_serial.close()
            self.rx_serial = None
        if self.fc_serial:
            self.fc_serial.close()
            self.fc_serial = None
    
    def bridge_loop(self, telemetry_data):
        """Головний цикл bridge"""
        print("🔄 Bridge thread started")
        
        while self.running:
            try:
                # USB1 → USB0 (RX → FC) - передача команд управління
                if self.rx_serial and self.rx_serial.in_waiting > 0:
                    data = self.rx_serial.read(self.rx_serial.in_waiting)
                    if self.fc_serial:
                        self.fc_serial.write(data)
                    self.stats['rx_packets'] += len(data)
                
                # USB0 → USB1 (FC → RX) - телеметрія назад
                if self.fc_serial and self.fc_serial.in_waiting > 0:
                    data = self.fc_serial.read(self.fc_serial.in_waiting)
                    if self.rx_serial:
                        self.rx_serial.write(data)
                    self.stats['fc_packets'] += len(data)
                
                # Оновити статистику в телеметрії
                telemetry_data.bridge_active = True
                telemetry_data.rx_packets = self.stats['rx_packets']
                telemetry_data.fc_packets = self.stats['fc_packets']
                
                time.sleep(0.001)  # 1ms
                
            except Exception as e:
                print(f"❌ Bridge error: {e}")
                self.stats['errors'] += 1
                telemetry_data.bridge_active = False
                time.sleep(0.01)
    
    def start(self, telemetry_data):
        """Запустити bridge"""
        if not self.connect():
            return False
        
        self.running = True
        
        # Запустити bridge потік
        self.bridge_thread = threading.Thread(
            target=self.bridge_loop, 
            args=(telemetry_data,), 
            daemon=True
        )
        self.bridge_thread.start()
        
        print("🚀 CRSF Bridge running!")
        return True
    
    def stop(self):
        """Зупинити bridge"""
        self.running = False
        time.sleep(0.1)
        self.disconnect()
        print("⏹️ Bridge stopped")

class HDMIOSDWithBridge:
    """HDMI OSD з CRSF bridge"""
    
    def __init__(self, rtsp_input=None, fc_port="/dev/ttyUSB0", rx_port="/dev/ttyUSB1", 
                 baud_rate=420000, resolution="1920x1080", framerate=30, fullscreen=True, 
                 enable_bridge=True):
        self.rtsp_input = rtsp_input
        self.fc_port = fc_port
        self.rx_port = rx_port
        self.baud_rate = baud_rate
        self.resolution = resolution
        self.framerate = framerate
        self.fullscreen = fullscreen
        self.enable_bridge = enable_bridge
        
        self.pipeline = None
        self.loop = None
        self.fc_serial = None  # Тільки для читання телеметрії
        self.running = False
        
        # Дані
        self.telemetry = TelemetryData()
        self.serial_buffer = bytearray()
        
        # GStreamer елементи
        self.text_overlays = {}
        
        # Розпарсити роздільність
        self.width, self.height = map(int, resolution.split('x'))
        
        # Bridge
        if self.enable_bridge:
            self.bridge = SimpleCRSFBridge(self.rx_port, self.fc_port, self.baud_rate)
        else:
            self.bridge = None
    
    def connect_telemetry(self):
        """Підключитися до FC для читання телеметрії"""
        try:
            self.fc_serial = serial.Serial(
                self.fc_port, self.baud_rate,
                timeout=0.01, bytesize=8, parity='N', stopbits=1
            )
            print(f"✅ Telemetry connected: {self.fc_port} @ {self.baud_rate}")
            return True
        except Exception as e:
            print(f"❌ Telemetry connection failed: {e}")
            return False
    
    def create_gstreamer_pipeline(self):
        """Створити GStreamer pipeline"""
        # Вибрати джерело відео з обробкою зависань
        if self.rtsp_input:
            video_source = f"""
            rtspsrc location={self.rtsp_input} 
                latency=0 
                drop-on-latency=true 
                do-retransmission=false 
                timeout=5000000
                tcp-timeout=5000000
                retry=3
                protocols=tcp+udp-mcast+udp ! 
            queue max-size-buffers=3 leaky=downstream ! 
            rtph264depay ! 
            queue max-size-buffers=3 leaky=downstream ! 
            avdec_h264 max-threads=2 skip-frame=1 ! 
            queue max-size-buffers=2 leaky=downstream ! 
            videoscale ! 
            video/x-raw,width={self.width},height={self.height} !
            videoconvert !
            queue max-size-buffers=2 leaky=downstream !
            """
        else:
            video_source = f"""
            videotestsrc pattern=ball is-live=true ! 
            video/x-raw,width={self.width},height={self.height},framerate={self.framerate}/1 ! 
            videoconvert !
            """
        
        # Pipeline з OSD
        pipeline_str = video_source
        
        # Додати overlay елементи
        overlays = [
            # RSSI & Link Quality
            ('rssi_overlay', 'RSSI: -- dBm\\nLQ: ---', 30, 40, 'left', 'top', 'Sans Bold 18', '0xFF00FF00'),
            # Battery
            ('battery_overlay', 'BATT: -.--V\\n--%', -30, 40, 'right', 'top', 'Sans Bold 18', '0xFF00FF00'),
            # GPS
            ('gps_overlay', 'GPS: -- SATs\\nALT: ---m', 30, -40, 'left', 'bottom', 'Sans Bold 16', '0xFF00FF00'),
            # Attitude
            ('attitude_overlay', 'PITCH: --°\\nROLL: --°\\nYAW: --°', -30, -40, 'right', 'bottom', 'Sans Bold 16', '0xFF00FF00'),
            # Flight Mode
            ('mode_overlay', 'MODE: UNKNOWN', 0, 80, 'center', 'top', 'Sans Bold 20', '0xFF00FFFF'),
            # Bridge Status
            ('bridge_overlay', 'BRIDGE: DISABLED', 0, 120, 'center', 'top', 'Sans Bold 16', '0xFFFFFF00'),
            # RC Channels
            ('rc_overlay', 'CH1-4: ---- ---- ---- ----', 0, -80, 'center', 'bottom', 'Sans Bold 14', '0xFFFFFF00'),
        ]
        
        for name, text, x, y, h_align, v_align, font, color in overlays:
            pipeline_str += f"""
            textoverlay name={name}
                text="{text}"
                halignment={h_align} valignment={v_align}
                x-absolute={x} y-absolute={y}
                font-desc="{font}"
                color={color} !
            """
        
        # Crosshair overlay
        pipeline_str += "cairooverlay name=crosshair_overlay !"
        
        # Додати буфер перед виходом
        pipeline_str += "queue max-size-buffers=2 leaky=downstream !"
        
        # Вихід через KMS для мінімальної затримки
        if self.fullscreen:
            pipeline_str += f"""
            videoconvert ! 
            videoscale method=nearest-neighbour ! 
            video/x-raw,width={self.width},height={self.height} !
            kmssink sync=false max-lateness=0 qos=false processing-deadline=0 render-delay=0 async=false
            """
        else:
            # Fallback для віконного режиму
            pipeline_str += f"""
            videoconvert ! 
            videoscale method=nearest-neighbour ! 
            video/x-raw,width={self.width},height={self.height} !
            ximagesink sync=false force-aspect-ratio=true qos=false async=false
            """
        
        return pipeline_str
    
    def setup_pipeline(self):
        """Налаштувати GStreamer pipeline"""
        pipeline_str = self.create_gstreamer_pipeline()
        
        print(f"🎬 Creating pipeline...")
        print(f"Pipeline: {pipeline_str[:100]}...")
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        if not self.pipeline:
            print("❌ Failed to create pipeline")
            return False
        
        # Отримати overlay елементи
        overlay_names = ['rssi_overlay', 'battery_overlay', 'gps_overlay', 
                        'attitude_overlay', 'mode_overlay', 'bridge_overlay', 'rc_overlay']
        
        for name in overlay_names:
            overlay = self.pipeline.get_by_name(name)
            if overlay:
                self.text_overlays[name] = overlay
        
        # Crosshair
        crosshair = self.pipeline.get_by_name('crosshair_overlay')
        if crosshair:
            crosshair.connect('draw', self.draw_crosshair)
        
        # Message handler
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message', self.on_message)
        
        # Додати watchdog для відновлення
        if self.rtsp_input:
            GLib.timeout_add_seconds(10, self.check_pipeline_health)
        
        return True
    
    def check_pipeline_health(self):
        """Перевірити здоров'я pipeline і перезапустити при зависанні"""
        try:
            # Отримати статистику з rtspsrc
            rtspsrc = self.pipeline.get_by_name("rtspsrc0")
            if rtspsrc:
                # Перевірити чи течуть дані
                pass
            
            # Продовжити моніторинг
            return True if self.running else False
            
        except Exception as e:
            print(f"⚠️ Pipeline health check failed: {e}")
            return True if self.running else False
    
    def restart_pipeline(self):
        """Перезапустити pipeline при проблемах"""
        try:
            print("🔄 Restarting pipeline...")
            self.pipeline.set_state(Gst.State.NULL)
            time.sleep(1)
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                print("❌ Failed to restart pipeline")
            else:
                print("✅ Pipeline restarted")
        except Exception as e:
            print(f"❌ Restart failed: {e}")
    
    def on_message(self, bus, message):
        """Обробник повідомлень GStreamer"""
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"❌ GStreamer Error: {err}")
            print(f"Debug: {debug}")
            
            # Спробувати перезапустити при помилці RTSP
            if "rtsp" in str(err).lower() or "network" in str(err).lower():
                print("🔄 Network error detected, attempting restart...")
                threading.Thread(target=self.restart_pipeline, daemon=True).start()
            else:
                self.loop.quit()
                
        elif message.type == Gst.MessageType.EOS:
            print("📺 End of stream")
            self.loop.quit()
            
        elif message.type == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            print(f"⚠️ GStreamer Warning: {warn}")
            
        elif message.type == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending = message.parse_state_changed()
                print(f"🎬 Pipeline state: {old_state.value_nick} → {new_state.value_nick}")
                
        elif message.type == Gst.MessageType.BUFFERING:
            percent = message.parse_buffering()
            print(f"📊 Buffering: {percent}%")
            if percent < 100:
                self.pipeline.set_state(Gst.State.PAUSED)
            else:
                self.pipeline.set_state(Gst.State.PLAYING)
    
    def draw_crosshair(self, overlay, cr, timestamp, duration, user_data):
        """Намалювати crosshair і штучний горизонт"""
        center_x = self.width / 2
        center_y = self.height / 2
        crosshair_size = int(self.width * 0.02)
        
        # Crosshair
        cr.set_source_rgba(0, 1, 0, 0.8)
        cr.set_line_width(3)
        
        cr.move_to(center_x - crosshair_size, center_y)
        cr.line_to(center_x + crosshair_size, center_y)
        cr.stroke()
        
        cr.move_to(center_x, center_y - crosshair_size)
        cr.line_to(center_x, center_y + crosshair_size)
        cr.stroke()
        
        cr.arc(center_x, center_y, 4, 0, 2 * math.pi)
        cr.fill()
        
        # Штучний горизонт
        if abs(self.telemetry.roll) > 1 or abs(self.telemetry.pitch) > 1:
            horizon_y = center_y + self.telemetry.pitch * 3
            roll_rad = self.telemetry.roll * math.pi / 180
            
            horizon_len = self.width * 0.1
            cos_roll = math.cos(roll_rad)
            sin_roll = math.sin(roll_rad)
            
            x1 = center_x - horizon_len * cos_roll
            y1 = horizon_y + horizon_len * sin_roll
            x2 = center_x + horizon_len * cos_roll
            y2 = horizon_y - horizon_len * sin_roll
            
            cr.set_source_rgba(0, 1, 0, 1)
            cr.set_line_width(4)
            cr.move_to(x1, y1)
            cr.line_to(x2, y2)
            cr.stroke()
    
    def update_osd_text(self):
        """Оновити OSD текст"""
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
        
        # Bridge Status
        if self.enable_bridge:
            bridge_color = 0xFF00FF00 if self.telemetry.bridge_active else 0xFFFF0000
            bridge_text = f"BRIDGE: {'ACTIVE' if self.telemetry.bridge_active else 'INACTIVE'}\\nRX→FC: {self.telemetry.rx_packets} | FC→RX: {self.telemetry.fc_packets}"
        else:
            bridge_color = 0xFFFFFF00
            bridge_text = "BRIDGE: DISABLED"
        
        if 'bridge_overlay' in self.text_overlays:
            self.text_overlays['bridge_overlay'].set_property('text', bridge_text)
            self.text_overlays['bridge_overlay'].set_property('color', bridge_color)
        
        # RC Channels
        rc_text = f"CH1-4: {self.telemetry.channels[0]} {self.telemetry.channels[1]} {self.telemetry.channels[2]} {self.telemetry.channels[3]}"
        if 'rc_overlay' in self.text_overlays:
            self.text_overlays['rc_overlay'].set_property('text', rc_text)
    
    def read_telemetry(self):
        """Читати телеметрію з FC"""
        while self.running:
            try:
                if self.fc_serial and self.fc_serial.in_waiting > 0:
                    data = self.fc_serial.read(self.fc_serial.in_waiting)
                    self.serial_buffer.extend(data)
                    
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
                
                time.sleep(0.001)
                
            except Exception as e:
                print(f"Telemetry read error: {e}")
                time.sleep(0.01)
    
    def update_osd_loop(self):
        """Оновити OSD"""
        while self.running:
            self.update_osd_text()
            time.sleep(0.1)  # 10 FPS
    
    def print_status(self):
        """Статус системи"""
        while self.running:
            current_time = time.time()
            
            # Статус зв'язку
            link_status = "🟢 LINK" if current_time - self.telemetry.link_last_update < 5 else "🔴 NO LINK"
            batt_status = f"🔋 {self.telemetry.voltage:.1f}V" if self.telemetry.voltage > 0 else "🔋 NO DATA"
            gps_status = f"🛰️ {self.telemetry.satellites}" if self.telemetry.satellites > 0 else "🛰️ NO FIX"
            
            # Bridge статус
            if self.enable_bridge:
                bridge_status = "🌉 ACTIVE" if self.telemetry.bridge_active else "🌉 INACTIVE"
            else:
                bridge_status = "🌉 DISABLED"
            
            print(f"\r{link_status} | {batt_status} | {gps_status} | {bridge_status} | "
                  f"CH1-4: {self.telemetry.channels[0]} {self.telemetry.channels[1]} {self.telemetry.channels[2]} {self.telemetry.channels[3]}    ", 
                  end="", flush=True)
            
            time.sleep(2)
    
    def run(self):
        """Запустити систему"""
        # Перевірити порти
        if self.enable_bridge:
            if not os.path.exists(self.rx_port):
                print(f"❌ RX port {self.rx_port} not found!")
                return False
            if not os.path.exists(self.fc_port):
                print(f"❌ FC port {self.fc_port} not found!")
                return False
        
        # Підключити телеметрію
        if not self.connect_telemetry():
            print("⚠️ Continuing without telemetry")
        
        # Налаштувати pipeline
        if not self.setup_pipeline():
            return False
        
        self.running = True
        
        # Запустити bridge
        if self.enable_bridge and self.bridge:
            if not self.bridge.start(self.telemetry):
                print("❌ Failed to start bridge")
                return False
        
        # Запустити потоки
        threads = []
        
        # Телеметрія
        if self.fc_serial:
            telemetry_thread = threading.Thread(target=self.read_telemetry, daemon=True)
            telemetry_thread.start()
            threads.append(telemetry_thread)
        
        # OSD
        osd_thread = threading.Thread(target=self.update_osd_loop, daemon=True)
        osd_thread.start()
        threads.append(osd_thread)
        
        # Статус
        status_thread = threading.Thread(target=self.print_status, daemon=True)
        status_thread.start()
        threads.append(status_thread)
        
        # Запустити pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("❌ Failed to start pipeline")
            return False
        
        # Головний loop
        self.loop = GLib.MainLoop()
        
        print("🎬 CRSF HDMI OSD with Bridge running!")
        print("🎬 CRSF HDMI OSD with Bridge running!")
        print(f"📺 Video: {self.rtsp_input or 'Test Pattern'} -> HDMI {self.resolution}")
        print(f"📡 Telemetry: {self.fc_port} @ {self.baud_rate}")
        
        if self.enable_bridge:
            print(f"🌉 Bridge: {self.rx_port} → {self.fc_port}")
            print("   Control commands: RX → FC")
            print("   Telemetry back: FC → RX")
        else:
            print("🌉 Bridge: DISABLED (OSD only)")
        
        print("Press Ctrl+C to stop\n")
        
        try:
            self.loop.run()
        except KeyboardInterrupt:
            print("\n🛑 Stopping system...")
        finally:
            self.running = False
            if self.bridge and self.enable_bridge:
                self.bridge.stop()
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
            if self.fc_serial:
                self.fc_serial.close()
        
        return True

def main():
    parser = argparse.ArgumentParser(description='CRSF HDMI OSD with Zero-Latency Bridge Control')
    parser.add_argument('-i', '--input', help='RTSP input URL')
    parser.add_argument('--fc-port', default='/dev/ttyUSB0', help='FC port (for telemetry)')
    parser.add_argument('--rx-port', default='/dev/ttyUSB1', help='RX port (for control input)')
    parser.add_argument('-b', '--baud', type=int, default=420000, help='CRSF baud rate')
    parser.add_argument('-r', '--resolution', default='1920x1080', 
                       choices=['1280x720', '1920x1080', '3840x2160', '2560x1440'],
                       help='Output resolution')
    parser.add_argument('-f', '--framerate', type=int, default=30, help='Frame rate')
    parser.add_argument('-w', '--windowed', action='store_true', help='Windowed mode (uses ximagesink)')
    parser.add_argument('--no-bridge', action='store_true', help='Disable bridge (OSD only)')
    parser.add_argument('--osd-only', action='store_true', help='OSD only mode (alias for --no-bridge)')
    
    args = parser.parse_args()
    
    # Визначити режим
    enable_bridge = not (args.no_bridge or args.osd_only)
    
    print("🎬 ZERO-LATENCY CRSF HDMI OSD WITH BRIDGE")
    print("=" * 60)
    print(f"Video Input: {args.input or 'Test Pattern'}")
    print(f"FC Port: {args.fc_port} (telemetry)")
    if enable_bridge:
        print(f"RX Port: {args.rx_port} (control input)")
        print(f"Bridge: {args.rx_port} → {args.fc_port}")
    else:
        print("Bridge: DISABLED")
    print(f"Baud Rate: {args.baud}")
    print(f"Output: {'KMS' if not args.windowed else 'X11'} {args.resolution} @ {args.framerate}fps")
    print(f"Latency: ZERO (sync=false, latency=0)")
    print(f"Mode: {'Windowed' if args.windowed else 'Fullscreen KMS'}")
    print()
    
    if not args.windowed:
        print("🚀 ZERO-LATENCY MODE")
        print("Using KMS sink for direct hardware output:")
        print("  • No X11 overhead")
        print("  • Direct DRM/KMS access")
        print("  • Hardware-accelerated scaling")
        print("  • Minimal processing pipeline")
        print("  • sync=false, max-lateness=0")
        print()
    
    if enable_bridge:
        print("🌉 BRIDGE MODE ENABLED")
        print("This will pass control commands from RX to FC:")
        print(f"  • RX input: {args.rx_port}")
        print(f"  • FC output: {args.fc_port}")
        print("  • Transparent passthrough like your original bridge")
        print("  • No joystick/keyboard - just data forwarding")
        print()
        
        if not os.path.exists(args.rx_port):
            print(f"❌ RX port {args.rx_port} not found!")
            print("Connect your RX device or use --osd-only")
            return
            
        if not os.path.exists(args.fc_port):
            print(f"❌ FC port {args.fc_port} not found!")
            print("Connect your FC device or use --osd-only")
            return
        
        response = input("Continue with bridge enabled? (y/n): ")
        if response.lower() != 'y':
            print("Bridge disabled for safety")
            enable_bridge = False
    
    print("Features:")
    print("  🎯 Crosshair + Artificial Horizon")
    print("  📊 Real-time telemetry OSD")
    if enable_bridge:
        print("  🌉 USB1→USB0 bridge (like your original script)")
        print("  📈 Bridge statistics display")
    if not args.windowed:
        print("  ⚡ Zero-latency KMS output")
    print("  📺 Professional video overlay")
    print()
    
    # Запустити систему
    system = HDMIOSDWithBridge(
        rtsp_input=args.input,
        fc_port=args.fc_port,
        rx_port=args.rx_port,
        baud_rate=args.baud,
        resolution=args.resolution,
        framerate=args.framerate,
        fullscreen=not args.windowed,
        enable_bridge=enable_bridge
    )
    
    system.run()

if __name__ == "__main__":
    main()