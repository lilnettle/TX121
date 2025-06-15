#!/usr/bin/env python3
"""
Simple Video + CRSF Bridge - мінімальний overhead
GStreamer як subprocess + простий bridge
"""

import subprocess
import argparse
import os
import time
import threading
import signal
import sys
import serial

class SimpleCRSFBridge:
    """Простий CRSF bridge"""
    
    def __init__(self, rx_port="/dev/ttyUSB1", fc_port="/dev/ttyUSB0", baud_rate=420000):
        self.rx_port = rx_port
        self.fc_port = fc_port
        self.baud_rate = baud_rate
        self.fallback_baud = 115200
        
        self.rx_serial = None
        self.fc_serial = None
        self.running = False
        self.bridge_thread = None
        
        self.stats = {'rx_packets': 0, 'fc_packets': 0, 'errors': 0}
        
    def connect(self):
        """Підключитися"""
        for baud in [self.baud_rate, self.fallback_baud]:
            try:
                print(f"🔌 Trying bridge at {baud} baud...")
                
                self.rx_serial = serial.Serial(self.rx_port, baud, timeout=0.01)
                self.fc_serial = serial.Serial(self.fc_port, baud, timeout=0.01)
                
                print(f"✅ Bridge connected at {baud} baud")
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
    
    def bridge_loop(self):
        """Головний цикл bridge"""
        print("🔄 Bridge thread started")
        
        while self.running:
            try:
                # USB1 → USB0 (RX → FC)
                if self.rx_serial and self.rx_serial.in_waiting > 0:
                    data = self.rx_serial.read(self.rx_serial.in_waiting)
                    if self.fc_serial:
                        self.fc_serial.write(data)
                    self.stats['rx_packets'] += len(data)
                
                # USB0 → USB1 (FC → RX)
                if self.fc_serial and self.fc_serial.in_waiting > 0:
                    data = self.fc_serial.read(self.fc_serial.in_waiting)
                    if self.rx_serial:
                        self.rx_serial.write(data)
                    self.stats['fc_packets'] += len(data)
                
                time.sleep(0.001)  # 1ms
                
            except Exception as e:
                print(f"❌ Bridge error: {e}")
                self.stats['errors'] += 1
                time.sleep(0.01)
    
    def start(self):
        """Запустити bridge"""
        if not self.connect():
            return False
        
        self.running = True
        self.bridge_thread = threading.Thread(target=self.bridge_loop, daemon=True)
        self.bridge_thread.start()
        
        print("🚀 CRSF Bridge running!")
        return True
    
    def stop(self):
        """Зупинити bridge"""
        self.running = False
        if self.bridge_thread:
            self.bridge_thread.join(timeout=1)
        self.disconnect()
        print("⏹️ Bridge stopped")

class SimpleVideoWithBridge:
    """Простий відеоплеєр з bridge"""
    
    def __init__(self):
        self.video_process = None
        self.bridge = None
        self.running = False
        
    def build_gstreamer_cmd(self, rtsp_url, resolution="1280x720", windowed=False, port_range="60000"):
        """Побудувати GStreamer команду (точно як працююча)"""
        width, height = resolution.split('x')
        
        if rtsp_url:
            if windowed:
                cmd = [
                    'gst-launch-1.0',
                    'rtspsrc', f'location={rtsp_url}', f'port-range={port_range}', 'protocols=tcp',
                    '!', 'decodebin',
                    '!', 'videoscale',
                    '!', f'video/x-raw,width={width},height={height}',
                    '!', 'videoconvert',
                    '!', 'ximagesink', 'sync=false'
                ]
            else:
                cmd = [
                    'gst-launch-1.0',
                    'rtspsrc', f'location={rtsp_url}', f'port-range={port_range}', 'protocols=tcp',
                    '!', 'decodebin',
                    '!', 'videoscale', 
                    '!', f'video/x-raw,width={width},height={height}',
                    '!', 'videoconvert',
                    '!', 'kmssink', 'sync=false'
                ]
        else:
            # Test pattern
            if windowed:
                cmd = [
                    'gst-launch-1.0',
                    'videotestsrc', 'pattern=ball',
                    '!', f'video/x-raw,width={width},height={height}',
                    '!', 'videoconvert',
                    '!', 'ximagesink', 'sync=false'
                ]
            else:
                cmd = [
                    'gst-launch-1.0',
                    'videotestsrc', 'pattern=ball',
                    '!', f'video/x-raw,width={width},height={height}',
                    '!', 'videoconvert',
                    '!', 'kmssink', 'sync=false'
                ]
        
        return cmd
    
    def print_status(self):
        """Статус системи"""
        while self.running:
            if self.bridge:
                bridge_status = "🌉 ACTIVE" 
                bridge_info = f"RX→FC: {self.bridge.stats['rx_packets']} | FC→RX: {self.bridge.stats['fc_packets']}"
                if self.bridge.stats['errors'] > 0:
                    bridge_info += f" | Errors: {self.bridge.stats['errors']}"
            else:
                bridge_status = "🌉 DISABLED"
                bridge_info = ""
            
            video_status = "📺 RUNNING" if self.video_process and self.video_process.poll() is None else "📺 STOPPED"
            
            print(f"\r{video_status} | {bridge_status} | {bridge_info}    ", 
                  end="", flush=True)
            
            time.sleep(2)
    
    def run(self, rtsp_url, resolution="1280x720", windowed=False, port_range="60000", 
            enable_bridge=False, rx_port="/dev/ttyUSB1", fc_port="/dev/ttyUSB0", baud_rate=420000):
        """Запустити систему"""
        
        print("🎬 SIMPLE VIDEO + CRSF BRIDGE")
        print("=" * 45)
        print(f"Video Input: {rtsp_url or 'Test Pattern'}")
        print(f"Output: {'X11 Window' if windowed else 'KMS Fullscreen'}")
        print(f"Resolution: {resolution}")
        if rtsp_url:
            print(f"Port Range: {port_range}")
        if enable_bridge:
            print(f"Bridge: {rx_port} → {fc_port} @ {baud_rate}")
        else:
            print("Bridge: DISABLED")
        print()
        
        # Перевірити порти bridge
        if enable_bridge:
            if not os.path.exists(rx_port):
                print(f"❌ RX port {rx_port} not found!")
                return False
            if not os.path.exists(fc_port):
                print(f"❌ FC port {fc_port} not found!")
                return False
        
        self.running = True
        
        # Запустити bridge
        if enable_bridge:
            self.bridge = SimpleCRSFBridge(rx_port, fc_port, baud_rate)
            if not self.bridge.start():
                print("❌ Failed to start bridge")
                return False
        
        # Побудувати команду відео
        cmd = self.build_gstreamer_cmd(rtsp_url, resolution, windowed, port_range)
        
        print("Video Command:")
        print(' '.join(cmd))
        print()
        print("Press Ctrl+C to stop")
        print()
        
        # Запустити статус
        status_thread = threading.Thread(target=self.print_status, daemon=True)
        status_thread.start()
        
        # Запустити відео
        try:
            self.video_process = subprocess.Popen(cmd,
                                                stdout=subprocess.PIPE,
                                                stderr=subprocess.PIPE,
                                                text=True)
            
            # Чекати завершення
            while self.running and self.video_process.poll() is None:
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping...")
        finally:
            self.stop()
        
        return True
    
    def stop(self):
        """Зупинити систему"""
        self.running = False
        
        if self.bridge:
            self.bridge.stop()
        
        if self.video_process:
            try:
                self.video_process.terminate()
                time.sleep(1)
                if self.video_process.poll() is None:
                    self.video_process.kill()
            except:
                pass

def signal_handler(sig, frame):
    """Обробник сигналів"""
    print('\n🛑 Signal received, stopping...')
    sys.exit(0)

def main():
    parser = argparse.ArgumentParser(description='Simple Video + CRSF Bridge - Direct GStreamer')
    parser.add_argument('-i', '--input', help='RTSP input URL')
    parser.add_argument('-r', '--resolution', default='1280x720', 
                       choices=['640x480', '848x480', '1280x720', '1920x1080'],
                       help='Output resolution')
    parser.add_argument('-w', '--windowed', action='store_true', help='Windowed mode')
    parser.add_argument('--port-range', default='60000', help='RTSP port range')
    parser.add_argument('--no-bridge', action='store_true', help='Disable bridge')
    parser.add_argument('--fc-port', default='/dev/ttyUSB0', help='FC port')
    parser.add_argument('--rx-port', default='/dev/ttyUSB1', help='RX port')
    parser.add_argument('-b', '--baud', type=int, default=420000, help='CRSF baud rate')
    
    args = parser.parse_args()
    
    # Встановити обробник сигналів
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    enable_bridge = not args.no_bridge
    
    system = SimpleVideoWithBridge()
    system.run(
        rtsp_url=args.input,
        resolution=args.resolution,
        windowed=args.windowed,
        port_range=args.port_range,
        enable_bridge=enable_bridge,
        rx_port=args.rx_port,
        fc_port=args.fc_port,
        baud_rate=args.baud
    )

if __name__ == "__main__":
    main()