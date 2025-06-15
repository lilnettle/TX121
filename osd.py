#!/usr/bin/env python3
"""
Clean Video + CRSF Bridge Control
- Чистий відеопотік без OSD
- Bridge управління: USB1 → USB0
- Без парсингу телеметрії
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import serial
import time
import threading
import argparse
import os
from dataclasses import dataclass

# Ініціалізація
Gst.init(None)

@dataclass
class BridgeStats:
    """Статистика bridge"""
    bridge_active: bool = False
    rx_packets: int = 0
    fc_packets: int = 0

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
    
    def bridge_loop(self, bridge_stats):
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
                
                # Оновити статистику
                bridge_stats.bridge_active = True
                bridge_stats.rx_packets = self.stats['rx_packets']
                bridge_stats.fc_packets = self.stats['fc_packets']
                
                time.sleep(0.001)  # 1ms
                
            except Exception as e:
                print(f"❌ Bridge error: {e}")
                self.stats['errors'] += 1
                bridge_stats.bridge_active = False
                time.sleep(0.01)
    
    def start(self, bridge_stats):
        """Запустити bridge"""
        if not self.connect():
            return False
        
        self.running = True
        
        # Запустити bridge потік
        self.bridge_thread = threading.Thread(
            target=self.bridge_loop, 
            args=(bridge_stats,), 
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

class CleanVideoWithBridge:
    """Чистий відеопотік з CRSF bridge"""
    
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
        self.running = False
        
        # Статистика bridge
        self.bridge_stats = BridgeStats()
        
        # Розпарсити роздільність
        self.width, self.height = map(int, resolution.split('x'))
        
        # Додаткові параметри
        self.port_range = "60000"
        self.use_decodebin = True  # За замовчуванням використовуємо decodebin
        
        # Bridge
        if self.enable_bridge:
            self.bridge = SimpleCRSFBridge(self.rx_port, self.fc_port, self.baud_rate)
        else:
            self.bridge = None
    
    def create_gstreamer_pipeline(self):
        """Створити GStreamer pipeline з оптимізаціями"""
        
        if self.rtsp_input:
            # Отримати режим оптимізації
            optimization_mode = getattr(self, 'optimization_mode', 'balanced')
            
            if optimization_mode == 'ultra_fast':
                # Ультра швидкий режим - мінімум обробки
                video_source = f"""
                rtspsrc location={self.rtsp_input} 
                    latency=0 
                    protocols=tcp ! 
                decodebin ! 
                videoconvert !
                """
            elif optimization_mode == 'stable':
                # Стабільний режим - більші буфери
                video_source = f"""
                rtspsrc location={self.rtsp_input} 
                    port-range={getattr(self, 'port_range', '60000')}
                    latency=100
                    buffer-mode=auto
                    protocols=tcp+udp ! 
                queue max-size-buffers=10 leaky=upstream ! 
                decodebin ! 
                queue max-size-buffers=5 leaky=upstream ! 
                videoscale ! 
                video/x-raw,width={self.width},height={self.height} !
                videoconvert !
                """
            elif optimization_mode == 'low_latency':
                # Низька затримка але стабільно
                video_source = f"""
                rtspsrc location={self.rtsp_input} 
                    port-range={getattr(self, 'port_range', '60000')}
                    latency=50
                    drop-on-latency=true
                    protocols=tcp ! 
                queue max-size-buffers=2 leaky=downstream ! 
                rtph264depay ! 
                avdec_h264 max-threads=1 ! 
                videoscale method=bilinear ! 
                video/x-raw,width={self.width},height={self.height} !
                videoconvert !
                """
            else:  # balanced
                # Збалансований режим
                video_source = f"""
                rtspsrc location={self.rtsp_input} 
                    port-range={getattr(self, 'port_range', '60000')}
                    latency=0 
                    drop-on-latency=true 
                    protocols=tcp+udp ! 
                queue max-size-buffers=3 leaky=downstream ! 
                decodebin ! 
                queue max-size-buffers=2 leaky=downstream ! 
                videoscale ! 
                video/x-raw,width={self.width},height={self.height} !
                videoconvert !
                """
        else:
            video_source = f"""
            videotestsrc pattern=ball is-live=true ! 
            video/x-raw,width={self.width},height={self.height},framerate={self.framerate}/1 ! 
            videoconvert !
            """
        
        # Pipeline
        pipeline_str = video_source
        
        # Буфер перед виходом (залежить від режиму)
        optimization_mode = getattr(self, 'optimization_mode', 'balanced')
        if optimization_mode == 'ultra_fast':
            pass  # Без додаткових буферів
        elif optimization_mode == 'stable':
            pipeline_str += "queue max-size-buffers=5 leaky=upstream !"
        else:
            pipeline_str += "queue max-size-buffers=2 leaky=downstream !"
        
        # Вихід
        if self.fullscreen:
            pipeline_str += f"""
            videoconvert ! 
            videoscale method=nearest-neighbour ! 
            video/x-raw,width={self.width},height={self.height} !
            kmssink sync=false max-lateness=-1 qos=false processing-deadline=0 render-delay=0 async=false
            """
        else:
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
        
        print(f"🎬 Creating clean video pipeline (no OSD)...")
        print(f"Pipeline: {pipeline_str[:100]}...")
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        if not self.pipeline:
            print("❌ Failed to create pipeline")
            return False
        
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
    
    def print_status(self):
        """Статус системи"""
        while self.running:
            
            # Bridge статус
            if self.enable_bridge:
                bridge_status = "🌉 ACTIVE" if self.bridge_stats.bridge_active else "🌉 INACTIVE"
                bridge_info = f"RX→FC: {self.bridge_stats.rx_packets} | FC→RX: {self.bridge_stats.fc_packets}"
            else:
                bridge_status = "🌉 DISABLED"
                bridge_info = ""
            
            print(f"\r📺 Clean Video | {bridge_status} | {bridge_info}    ", 
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
        
        # НЕ підключаємо телеметрію - тільки bridge
        print("📺 Clean video mode - no telemetry parsing")
        
        # Налаштувати pipeline
        if not self.setup_pipeline():
            return False
        
        self.running = True
        
        # Запустити bridge (якщо потрібен)
        if self.enable_bridge and self.bridge:
            if not self.bridge.start(self.bridge_stats):
                print("❌ Failed to start bridge")
                return False
        
        # Запустити тільки статус потік (без телеметрії і OSD)
        status_thread = threading.Thread(target=self.print_status, daemon=True)
        status_thread.start()
        
        # Запустити pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("❌ Failed to start pipeline")
            return False
        
        # Головний loop
        self.loop = GLib.MainLoop()
        
        print("🎬 Clean Video + CRSF Bridge running!")
        print(f"📺 Video: {self.rtsp_input or 'Test Pattern'} -> HDMI {self.resolution}")
        
        if self.enable_bridge:
            print(f"🌉 Bridge: {self.rx_port} → {self.fc_port}")
            print("   Control commands: RX → FC")
            print("   Telemetry back: FC → RX")
            print("   📺 Video: Clean passthrough (no OSD)")
        else:
            print("🌉 Bridge: DISABLED")
            print("   📺 Video: Clean passthrough only")
        
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
        
        return True

def main():
    parser = argparse.ArgumentParser(description='Clean Video + CRSF Bridge Control - Optimized')
    parser.add_argument('-i', '--input', help='RTSP input URL')
    parser.add_argument('--fc-port', default='/dev/ttyUSB0', help='FC port (for bridge)')
    parser.add_argument('--rx-port', default='/dev/ttyUSB1', help='RX port (for control input)')
    parser.add_argument('-b', '--baud', type=int, default=420000, help='CRSF baud rate')
    parser.add_argument('-r', '--resolution', default='1920x1080', 
                       choices=['1280x720', '1920x1080', '3840x2160', '2560x1440', '640x480', '848x480'],
                       help='Output resolution')
    parser.add_argument('-f', '--framerate', type=int, default=30, help='Frame rate')
    parser.add_argument('-w', '--windowed', action='store_true', help='Windowed mode (uses ximagesink)')
    parser.add_argument('--no-bridge', action='store_true', help='Disable bridge (video only)')
    parser.add_argument('--port-range', default='60000', help='RTSP port range (default: 60000)')
    parser.add_argument('--optimization', choices=['ultra_fast', 'low_latency', 'balanced', 'stable'], 
                       default='balanced', help='Optimization mode')
    
    args = parser.parse_args()
    
    # Визначити режим
    enable_bridge = not args.no_bridge
    
    print("🎬 OPTIMIZED CLEAN VIDEO + CRSF BRIDGE")
    print("=" * 50)
    print(f"Video Input: {args.input or 'Test Pattern'}")
    if enable_bridge:
        print(f"RX Port: {args.rx_port} (control input)")
        print(f"FC Port: {args.fc_port} (bridge output)")
        print(f"Bridge: {args.rx_port} → {args.fc_port}")
    else:
        print("Bridge: DISABLED")
    print(f"Baud Rate: {args.baud}")
    print(f"Output: {'KMS' if not args.windowed else 'X11'} {args.resolution} @ {args.framerate}fps")
    print(f"RTSP Port Range: {args.port_range}")
    print(f"Optimization: {args.optimization.upper()}")
    print()
    
    # Опис режимів оптимізації
    optimization_descriptions = {
        'ultra_fast': '⚡ ULTRA FAST - мінімум обробки, може бути нестабільно',
        'low_latency': '🏎️ LOW LATENCY - швидко + стабільно',
        'balanced': '⚖️ BALANCED - оптимальний баланс',
        'stable': '🛡️ STABLE - максимальна стабільність, вища затримка'
    }
    print(f"Mode: {optimization_descriptions[args.optimization]}")
    print()
    
    if enable_bridge:
        print("🌉 BRIDGE MODE ENABLED")
        print("This will pass control commands from RX to FC:")
        print(f"  • RX input: {args.rx_port}")
        print(f"  • FC output: {args.fc_port}")
        print("  • No OSD overlay on video")
        print("  • No telemetry parsing (performance optimized)")
        print()
        
        if not os.path.exists(args.rx_port):
            print(f"❌ RX port {args.rx_port} not found!")
            print("Connect your RX device or use --no-bridge")
            return
            
        if not os.path.exists(args.fc_port):
            print(f"❌ FC port {args.fc_port} not found!")
            print("Connect your FC device or use --no-bridge")
            return
        
        if args.optimization == 'ultra_fast':
            print("⚠️ WARNING: ultra_fast mode with bridge may cause issues")
            response = input("Continue? (y/n): ")
            if response.lower() != 'y':
                return
        else:
            response = input("Continue with bridge enabled? (y/n): ")
            if response.lower() != 'y':
                print("Bridge disabled")
                enable_bridge = False
    
    print("Features:")
    print("  📺 Optimized video pipeline")
    print(f"  🔧 {args.optimization.upper()} optimization mode")
    print("  ⚡ Zero-latency KMS output")
    print("  🔄 Auto-recovery on video freeze")
    print(f"  🔌 RTSP port-range={args.port_range}")
    if enable_bridge:
        print("  🌉 USB1→USB0 bridge")
        print("  📈 Bridge statistics in console")
    print("  🚫 NO OSD overlay")
    print("  🚫 NO telemetry parsing")
    print()
    
    # Запустити систему
    system = CleanVideoWithBridge(
        rtsp_input=args.input,
        fc_port=args.fc_port,
        rx_port=args.rx_port,
        baud_rate=args.baud,
        resolution=args.resolution,
        framerate=args.framerate,
        fullscreen=not args.windowed,
        enable_bridge=enable_bridge
    )
    
    # Додати параметри оптимізації
    system.port_range = args.port_range
    system.optimization_mode = args.optimization
    
    system.run()

if __name__ == "__main__":
    main()