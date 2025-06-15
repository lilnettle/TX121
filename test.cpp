#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

int main() {
    std::cout << "🔍 Testing USB ports...\n";
    
    // Test USB1 (RX)
    int fd1 = open("/dev/ttyUSB1", O_RDONLY | O_NONBLOCK);
    if (fd1 < 0) {
        std::cout << "❌ Cannot open USB1: " << strerror(errno) << "\n";
        return 1;
    }
    std::cout << "✅ USB1 opened\n";
    
    // Test USB0 (FC)  
    int fd0 = open("/dev/ttyUSB0", O_RDONLY | O_NONBLOCK);
    if (fd0 < 0) {
        std::cout << "❌ Cannot open USB0: " << strerror(errno) << "\n";
        close(fd1);
        return 1;
    }
    std::cout << "✅ USB0 opened\n";
    
    char buffer[1000];
    int usb1_count = 0, usb0_count = 0;
    
    for (int i = 0; i < 100; i++) {
        // Check USB1
        ssize_t bytes1 = read(fd1, buffer, sizeof(buffer));
        if (bytes1 > 0) {
            usb1_count++;
            std::cout << "📡 USB1: " << bytes1 << " bytes\n";
        }
        
        // Check USB0
        ssize_t bytes0 = read(fd0, buffer, sizeof(buffer));
        if (bytes0 > 0) {
            usb0_count++;
            std::cout << "📡 USB0: " << bytes0 << " bytes\n";
        }
        
        usleep(50000); // 50ms
    }
    
    std::cout << "\n📊 Results:\n";
    std::cout << "USB1 reads: " << usb1_count << "\n";
    std::cout << "USB0 reads: " << usb0_count << "\n";
    
    close(fd1);
    close(fd0);
    return 0;
}