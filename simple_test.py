#!/usr/bin/env python3
"""
Найпростіший тест - відправляємо сирі байти
"""

import serial
import time

def test_raw_data():
    """Відправити прості тестові дані"""
    
    ports = ["/dev/ttyUSB0", "/dev/ttyFIQ0"]
    
    for port in ports:
        print(f"\n🧪 Testing RAW data on {port}")
        
        try:
            ser = serial.Serial(port, 115200, timeout=0.1)
            
            # Відправити прості байти
            test_patterns = [
                b'\xAA\x55\xAA\x55',  # Alternating pattern
                b'\xFF\x00\xFF\x00',  # High/Low pattern  
                b'\xC8\x18\x16\x00',  # CRSF header
                b'HELLO_FC',          # Text
            ]
            
            for i, pattern in enumerate(test_patterns):
                print(f"   📡 Sending pattern {i+1}: {pattern.hex()}")
                
                # Відправити 50 разів
                for _ in range(50):
                    ser.write(pattern)
                    time.sleep(0.02)
                
                time.sleep(1)
            
            ser.close()
            print(f"   ✅ Raw test completed")
            
        except Exception as e:
            print(f"   ❌ Failed: {e}")

def test_loopback():
    """Тест зворотнього зв'язку (якщо можливо)"""
    
    port = "/dev/ttyUSB0"
    
    print(f"\n🔄 Testing loopback on {port}")
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        
        # Відправити і спробувати прочитати
        test_data = b"TEST_LOOPBACK"
        
        ser.write(test_data)
        time.sleep(0.1)
        
        received = ser.read(100)
        
        if received:
            print(f"   📥 Received: {received}")
        else:
            print(f"   📭 No data received (normal for one-way)")
            
        ser.close()
        
    except Exception as e:
        print(f"   ❌ Loopback failed: {e}")

def main():
    print("🔧 Raw UART Test")
    print("=" * 30)
    print("This will send simple patterns to test basic UART communication")
    
    test_raw_data()
    test_loopback()
    
    print(f"\n💡 Next steps:")
    print(f"   1. Check if FC receives ANY data (CLI: tasks)")
    print(f"   2. Verify physical wiring")
    print(f"   3. Check voltage levels")
    print(f"   4. Try different UART pins on FC")

if __name__ == "__main__":
    main()