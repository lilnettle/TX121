#!/usr/bin/env python3
"""
Адаптований CRSF тестер на основі робочого коду
Використовує правильний алгоритм упаковки каналів
"""

import serial
import time
import argparse
from enum import IntEnum

CRSF_SYNC = 0xC8

class PacketsTypes(IntEnum):
    RC_CHANNELS_PACKED = 0x16

def crc8_dvb_s2(crc, a) -> int:
    """Точний CRC8 DVB-S2 як в робочому коді"""
    crc = crc ^ a
    for ii in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc = crc << 1
    return crc & 0xFF

def crc8_data(data) -> int:
    """Розрахунок CRC8 для масиву байтів"""
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc

def packCrsfToBytes(channels) -> bytes:
    """ПРАВИЛЬНА упаковка каналів як в робочому коді"""
    # channels is in CRSF format! (0-1984)
    # Values are packed little-endianish such that bits BA987654321 -> 87654321, 00000BA9
    # 11 bits per channel x 16 channels = 22 bytes
    if len(channels) != 16:
        raise ValueError('CRSF must have 16 channels')
    
    result = bytearray()
    destShift = 0
    newVal = 0
    
    for ch in channels:
        # Put the low bits in any remaining dest capacity
        newVal |= (ch << destShift) & 0xff
        result.append(newVal)

        # Shift the high bits down and place them into the next dest byte
        srcBitsLeft = 11 - 8 + destShift
        newVal = ch >> (11 - srcBitsLeft)
        
        # When there's at least a full byte remaining, consume that as well
        if srcBitsLeft >= 8:
            result.append(newVal & 0xff)
            newVal >>= 8
            srcBitsLeft -= 8

        # Next dest should be shifted up by the bits consumed
        destShift = srcBitsLeft

    return result

def channelsCrsfToChannelsPacket(channels) -> bytes:
    """Створити CRSF пакет каналів"""
    result = bytearray([CRSF_SYNC, 24, PacketsTypes.RC_CHANNELS_PACKED])  # 24 is packet length
    result += packCrsfToBytes(channels)
    result.append(crc8_data(result[2:]))
    return result

def pwm_to_crsf(pwm_value):
    """Конвертувати PWM (1000-2000) в CRSF формат (172-1811)"""
    if pwm_value < 1000:
        return 172
    elif pwm_value > 2000:
        return 1811
    else:
        # Лінійна конверсія з правильним масштабуванням
        return int(172 + (pwm_value - 1000) * (1811 - 172) / 1000)

def test_crsf_connection():
    """Тест CRSF підключення з різними швидкостями"""
    
    print("🚀 PROFESSIONAL CRSF TESTER")
    print("=" * 40)
    print("Using working CRSF implementation")
    
    # Швидкості для тестування
    speeds_to_test = [
        (921600, "High Speed"),
        (420000, "Betaflight Default"),
        (416666, "CRSF Rev10 Official"),
        (400000, "CRSF Rev7 Official"),
        (115200, "Standard Serial"),
    ]
    
    ports_to_test = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"]
    
    for port in ports_to_test:
        print(f"\n🔌 Testing port: {port}")
        
        for speed, description in speeds_to_test:
            print(f"\n📡 Testing {speed} baud ({description})")
            
            try:
                ser = serial.Serial(port, speed, timeout=0.1)
                print(f"  ✅ Opened {port} at {speed} baud")
                
                # Тестова послідовність з CRSF значеннями
                test_sequence = [
                    ([992, 992, 992, 992], "🟡 ALL CENTER"),
                    ([172, 992, 992, 992], "🔴 ROLL MIN"),
                    ([992, 992, 992, 992], "🟡 CENTER"),
                    ([1811, 992, 992, 992], "🔴 ROLL MAX"),
                    ([992, 992, 992, 992], "🟡 CENTER"),
                    ([992, 172, 992, 992], "🟢 PITCH MIN"),
                    ([992, 992, 992, 992], "🟡 CENTER"),
                    ([992, 1811, 992, 992], "🟢 PITCH MAX"),
                    ([992, 992, 992, 992], "🟡 CENTER"),
                    ([992, 992, 992, 172], "🔵 YAW MIN"),
                    ([992, 992, 992, 992], "🟡 CENTER"),
                    ([992, 992, 992, 1811], "🔵 YAW MAX"),
                    ([992, 992, 992, 992], "🟡 CENTER"),
                ]
                
                for crsf_channels, name in test_sequence:
                    # Заповнити всі 16 каналів
                    full_channels = crsf_channels + [992] * (16 - len(crsf_channels))
                    
                    # Створити пакет з правильним алгоритмом
                    packet = channelsCrsfToChannelsPacket(full_channels)
                    
                    print(f"    {name}")
                    
                    # Відправити пакет 50 разів (1 секунда)
                    for _ in range(50):
                        ser.write(packet)
                        time.sleep(0.02)  # 50 Hz
                    
                    time.sleep(0.3)  # Пауза між тестами
                
                ser.close()
                
                response = input(f"  ❓ Did channels move at {speed} baud? (y/n): ")
                if response.lower() == 'y':
                    print(f"  🎉 SUCCESS! {description} works on {port}")
                    return port, speed, description
                else:
                    print(f"  ❌ No movement at {speed} baud")
                    
            except Exception as e:
                print(f"  ❌ Failed to test {port} at {speed}: {e}")
        
        print(f"  ⏭️ Next port...")
    
    return None, None, None

def run_continuous_crsf_test(port, speed):
    """Безперервний тест з PWM значеннями"""
    
    print(f"\n🎮 CONTINUOUS CRSF TEST")
    print(f"Port: {port}, Speed: {speed}")
    print("Press Ctrl+C to stop")
    print("-" * 30)
    
    try:
        ser = serial.Serial(port, speed, timeout=0.01)
        cycle = 0
        
        while True:
            cycle += 1
            
            # Цикл тестів кожні 3 секунди
            phase = (cycle // 150) % 6
            
            if phase == 0:
                # Roll test - PWM 1000 to 2000
                pwm_roll = 1000 + (cycle % 150) * 1000 // 150
                pwm_channels = [pwm_roll, 1500, 1100, 1500]
                state = f"ROLL_SWEEP  {pwm_roll:4d}"
            elif phase == 1:
                # Pitch test
                pwm_pitch = 1000 + (cycle % 150) * 1000 // 150
                pwm_channels = [1500, pwm_pitch, 1100, 1500]
                state = f"PITCH_SWEEP {pwm_pitch:4d}"
            elif phase == 2:
                # Yaw test
                pwm_yaw = 1000 + (cycle % 150) * 1000 // 150
                pwm_channels = [1500, 1500, 1100, pwm_yaw]
                state = f"YAW_SWEEP   {pwm_yaw:4d}"
            elif phase == 3:
                # All minimum
                pwm_channels = [1000, 1000, 1100, 1000]
                state = "ALL_MIN     1000"
            elif phase == 4:
                # All maximum
                pwm_channels = [2000, 2000, 1100, 2000]
                state = "ALL_MAX     2000"
            else:
                # Center
                pwm_channels = [1500, 1500, 1100, 1500]
                state = "CENTER      1500"
            
            # Конвертувати PWM в CRSF
            crsf_channels = [pwm_to_crsf(pwm) for pwm in pwm_channels]
            crsf_channels.extend([992] * 12)  # Решта каналів в центр
            
            # Створити та відправити пакет
            packet = channelsCrsfToChannelsPacket(crsf_channels)
            ser.write(packet)
            
            if cycle % 25 == 0:
                print(f"\r🎮 {state} | CRSF: R={crsf_channels[0]:4d} P={crsf_channels[1]:4d} "
                      f"T={crsf_channels[2]:4d} Y={crsf_channels[3]:4d} | Cycle:{cycle:5d}", 
                      end="", flush=True)
            
            time.sleep(0.02)  # 50 Hz
            
    except KeyboardInterrupt:
        print(f"\n⏹️ Test stopped")
        ser.close()

def main():
    print("🔧 PROFESSIONAL CRSF TESTER")
    print("=" * 50)
    print("Based on working CRSF implementation")
    print("Uses correct channel packing algorithm")
    print()
    print("Betaflight should be configured:")
    print("  serialrx_provider = CRSF")
    print("  serial 1 64 (UART1 as Serial RX)")
    print()
    
    ready = input("❓ Ready to test? (y/n): ")
    if ready.lower() != 'y':
        return
    
    # Знайти робочу конфігурацію
    working_port, working_speed, description = test_crsf_connection()
    
    if working_port:
        print(f"\n🎉 CRSF IS WORKING!")
        print(f"   Port: {working_port}")
        print(f"   Speed: {working_speed} baud")
        print(f"   Type: {description}")
        print(f"\n✅ You can now build your FPV simulator!")
        
        # Безперервний тест
        cont = input(f"\n❓ Run continuous test with sweeping channels? (y/n): ")
        if cont.lower() == 'y':
            run_continuous_crsf_test(working_port, working_speed)
    else:
        print(f"\n❌ CRSF not working on any configuration")
        print(f"\n🔍 Final troubleshooting suggestions:")
        print(f"   1. Physical UART1 connection (TX1/RX1 pins)")
        print(f"   2. 3.3V logic levels (not 5V)")
        print(f"   3. Different FC UART port")
        print(f"   4. Check FC firmware supports CRSF")

if __name__ == "__main__":
    main()