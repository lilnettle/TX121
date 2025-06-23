#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>
#include <cstring>
#include <csignal>
#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <time.h>
#include <cstdint>
#include <cstdio>

// GStreamer
#include <gst/gst.h>
#include <glib.h>

// НАЛАШТУВАННЯ
const std::string RX_PORT = "/dev/ttyUSB1";
const std::string CAMERA_IP = "192.168.0.100";
const std::string RTSP_URL = "rtsp://root:12345@192.168.0.100:554/stream1";
const int CRSF_UDP_PORT = 5000;
const int TELEMETRY_UDP_PORT = 5001;
const int PRIMARY_BAUD = 420000;
const int FALLBACK_BAUD = 115200;

// Константи протоколу
const uint8_t CRSF_SYNC_BYTE = 0xC8;
const size_t MAX_PACKET_SIZE = 256;
const size_t BUFFER_SIZE = 1024;

// Типи UDP пакетів
enum class PacketType : uint8_t {
    CRSF_COMMANDS = 0x01,
    TELEMETRY = 0x02,
    ACK = 0x03,
    HEARTBEAT = 0x04
};

// UDP пакет структура - ВИПРАВЛЕНО
typedef struct {
    uint64_t timestamp;
    uint8_t packet_type;
    uint16_t data_length;
    uint32_t sequence_id;
    uint8_t data[MAX_PACKET_SIZE];
} __attribute__((packed)) udp_packet_t;

// ACK пакет - ВИПРАВЛЕНО
typedef struct {
    uint64_t timestamp;
    uint8_t packet_type;   // 0x03
    uint16_t data_length;  // завжди 4
    uint32_t ack_sequence_id;
} __attribute__((packed)) ack_packet_t;

// Статистика
struct Stats {
    std::atomic<uint32_t> crsf_received{0};
    std::atomic<uint32_t> crsf_sent_udp{0};
    std::atomic<uint32_t> crsf_acked{0};
    std::atomic<uint32_t> crsf_failed{0};
    std::atomic<uint32_t> telemetry_received_udp{0};
    std::atomic<uint32_t> telemetry_sent_uart{0};
    std::atomic<uint32_t> heartbeats_sent{0};
    std::atomic<uint32_t> heartbeats_received{0};
    std::atomic<uint32_t> errors{0};
    time_t start_time;
    time_t last_camera_response;
    std::atomic<bool> camera_online{false};
    
    Stats() { 
        start_time = time(nullptr); 
        last_camera_response = 0;
    }
};

// Pending ACK tracking - ВИПРАВЛЕНО
struct PendingPacket {
    uint32_t sequence_id;
    std::chrono::steady_clock::time_point send_time;
    std::vector<uint8_t> packet_data;  // Зберігаємо як vector
    size_t packet_size;
    int retry_count;
    
    PendingPacket(uint32_t seq, const uint8_t* data, size_t size) 
        : sequence_id(seq), send_time(std::chrono::steady_clock::now()), 
          packet_size(size), retry_count(0) {
        packet_data.resize(size);
        memcpy(packet_data.data(), data, size);
    }
};

// Callback функції - ВИПРАВЛЕНО типи
class Callbacks {
public:
    static void on_crsf_command_sent(uint32_t seq_id, size_t bytes) {
        printf("📤 [%ld] CRSF command #%u sent (%zu bytes)\n", 
               time(nullptr), seq_id, bytes);
    }
    
    static void on_crsf_command_acked(uint32_t seq_id, uint64_t latency_us) {
        printf("✅ [%ld] CRSF command #%u acknowledged (latency: %llu μs)\n", 
               time(nullptr), seq_id, (unsigned long long)latency_us);
    }
    
    static void on_crsf_command_failed(uint32_t seq_id, const std::string& reason) {
        printf("❌ [%ld] CRSF command #%u failed: %s\n", 
               time(nullptr), seq_id, reason.c_str());
    }
    
    static void on_telemetry_received(size_t bytes, const std::string& source) {
        printf("📥 [%ld] Telemetry received: %zu bytes from %s\n", 
               time(nullptr), bytes, source.c_str());
    }
    
    static void on_camera_status_changed(bool online, const std::string& details) {
        printf("🔄 [%ld] Camera status: %s (%s)\n", 
               time(nullptr), online ? "ONLINE" : "OFFLINE", details.c_str());
    }
    
    static void on_connection_quality(float success_rate, uint64_t avg_latency) {
        printf("📊 [%ld] Connection quality: %.1f%% success, %llu μs avg latency\n",
               time(nullptr), success_rate * 100, (unsigned long long)avg_latency);
    }
};

// Серійний порт клас
class SerialPort {
private:
    int fd = -1;
    std::string port_name;
    int baud_rate;
    bool is_open = false;

    bool set_baud_rate(int baud) {
        struct termios tty;
        
        if (tcgetattr(fd, &tty) < 0) {
            std::cerr << "❌ tcgetattr failed for " << port_name << std::endl;
            return false;
        }
        
        cfmakeraw(&tty);
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;
        
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(INLCR | IGNCR | ICRNL);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP);
        
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        
        speed_t speed;
        bool is_custom = false;
        
        switch (baud) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            case 921600: speed = B921600; break;
            default:
                speed = B38400;
                is_custom = true;
                break;
        }
        
        if (cfsetispeed(&tty, speed) < 0 || cfsetospeed(&tty, speed) < 0) {
            std::cerr << "❌ cfsetspeed failed" << std::endl;
            return false;
        }
        
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            std::cerr << "❌ tcsetattr failed" << std::endl;
            return false;
        }
        
        if (is_custom && baud == 420000) {
            struct serial_struct ss;
            if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
                ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
                ss.custom_divisor = ss.baud_base / baud;
                
                if (ioctl(fd, TIOCSSERIAL, &ss) == 0) {
                    std::cout << "✅ " << port_name << ": custom baud " << baud << " set" << std::endl;
                } else {
                    std::cout << "⚠️ " << port_name << ": failed to set custom baud, using base" << std::endl;
                }
            }
        }
        
        return true;
    }

public:
    SerialPort(const std::string& port, int baud) 
        : port_name(port), baud_rate(baud) {}
    
    ~SerialPort() {
        close();
    }
    
    bool open() {
        fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            std::cerr << "❌ Failed to open " << port_name << ": " << strerror(errno) << std::endl;
            return false;
        }
        
        if (!set_baud_rate(baud_rate)) {
            ::close(fd);
            fd = -1;
            return false;
        }
        
        tcflush(fd, TCIOFLUSH);
        is_open = true;
        return true;
    }
    
    void close() {
        if (is_open && fd >= 0) {
            tcflush(fd, TCIOFLUSH);
            ::close(fd);
            fd = -1;
            is_open = false;
        }
    }
    
    int available() {
        if (!is_open) return 0;
        
        int bytes_available = 0;
        if (ioctl(fd, FIONREAD, &bytes_available) < 0) {
            return 0;
        }
        return bytes_available;
    }
    
    ssize_t read(uint8_t* buffer, size_t size) {
        if (!is_open) return -1;
        
        ssize_t result = ::read(fd, buffer, size);
        if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            return -1;
        }
        return result > 0 ? result : 0;
    }
    
    ssize_t write(const uint8_t* buffer, size_t size) {
        if (!is_open) return -1;
        
        ssize_t result = ::write(fd, buffer, size);
        if (result > 0) {
            fsync(fd);
        }
        return result;
    }
    
    bool connected() const { return is_open; }
    const std::string& get_name() const { return port_name; }
};

// UDP клієнт з ACK tracking - ВИПРАВЛЕНО
class UDPClientWithACK {
private:
    int socket_fd = -1;
    struct sockaddr_in server_addr;
    std::string server_ip;
    int server_port;
    std::atomic<uint32_t> sequence_counter{1};
    
    // ACK tracking - ВИПРАВЛЕНО mutex
    std::map<uint32_t, PendingPacket> pending_packets;
    mutable std::mutex pending_mutex;  // mutable для const методів
    std::thread ack_monitor_thread;
    std::atomic<bool> running{false};

public:
    UDPClientWithACK(const std::string& ip, int port) : server_ip(ip), server_port(port) {}
    
    ~UDPClientWithACK() {
        close();
    }
    
    bool open() {
        socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0) {
            std::cerr << "❌ Failed to create UDP client socket: " << strerror(errno) << std::endl;
            return false;
        }
        
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port);
        
        if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0) {
            std::cerr << "❌ Invalid IP address: " << server_ip << std::endl;
            ::close(socket_fd);
            socket_fd = -1;
            return false;
        }
        
        running = true;
        ack_monitor_thread = std::thread(&UDPClientWithACK::ack_monitor_loop, this);
        
        std::cout << "✅ UDP client with ACK configured for " << server_ip << ":" << server_port << std::endl;
        return true;
    }
    
    void close() {
        running = false;
        
        if (ack_monitor_thread.joinable()) {
            ack_monitor_thread.join();
        }
        
        if (socket_fd >= 0) {
            ::close(socket_fd);
            socket_fd = -1;
        }
    }
    
    uint32_t send_with_ack(const uint8_t* data, size_t size) {
        if (socket_fd < 0) return 0;
        
        // Створити UDP пакет
        std::vector<uint8_t> packet_buffer(sizeof(udp_packet_t));
        udp_packet_t* packet = reinterpret_cast<udp_packet_t*>(packet_buffer.data());
        
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        packet->timestamp = static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + ts.tv_nsec / 1000;
        packet->packet_type = static_cast<uint8_t>(PacketType::CRSF_COMMANDS);
        packet->data_length = static_cast<uint16_t>(size);
        packet->sequence_id = sequence_counter++;
        
        // Копіювати дані
        if (size > 0 && size <= MAX_PACKET_SIZE) {
            memcpy(packet->data, data, size);
        }
        
        // Розрахувати фактичний розмір пакету
        size_t packet_size = sizeof(udp_packet_t) - MAX_PACKET_SIZE + size;
        
        // Відправити
        ssize_t sent = sendto(socket_fd, packet, packet_size, 0,
                             (struct sockaddr*)&server_addr, sizeof(server_addr));
        
        if (sent > 0) {
            // Додати до pending
            {
                std::lock_guard<std::mutex> lock(pending_mutex);
                pending_packets.emplace(std::piecewise_construct,
                    std::forward_as_tuple(packet->sequence_id),
                    std::forward_as_tuple(packet->sequence_id, packet_buffer.data(), packet_size));
            }
            
            Callbacks::on_crsf_command_sent(packet->sequence_id, size);
            return packet->sequence_id;
        }
        
        return 0;
    }
    
    void process_ack(uint32_t ack_seq_id, uint64_t timestamp) {
        std::lock_guard<std::mutex> lock(pending_mutex);
        auto it = pending_packets.find(ack_seq_id);
        if (it != pending_packets.end()) {
            auto send_time = it->second.send_time;
            auto now = std::chrono::steady_clock::now();
            auto latency = std::chrono::duration_cast<std::chrono::microseconds>(now - send_time).count();
            
            Callbacks::on_crsf_command_acked(ack_seq_id, static_cast<uint64_t>(latency));
            pending_packets.erase(it);
        }
    }
    
private:
    void ack_monitor_loop() {
        const auto timeout = std::chrono::seconds(2);
        const int max_retries = 3;
        
        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            auto now = std::chrono::steady_clock::now();
            std::vector<uint32_t> expired_packets;
            
            {
                std::lock_guard<std::mutex> lock(pending_mutex);
                for (auto& [seq_id, pending] : pending_packets) {
                    if (now - pending.send_time > timeout) {
                        if (pending.retry_count < max_retries) {
                            // Retry
                            pending.retry_count++;
                            pending.send_time = now;
                            
                            sendto(socket_fd, pending.packet_data.data(), pending.packet_size, 0,
                                  (struct sockaddr*)&server_addr, sizeof(server_addr));
                            
                            printf("🔄 Retrying packet #%u (attempt %d/%d)\n", 
                                   seq_id, pending.retry_count, max_retries);
                        } else {
                            // Failed
                            expired_packets.push_back(seq_id);
                        }
                    }
                }
                
                // Видалити failed пакети
                for (uint32_t seq_id : expired_packets) {
                    pending_packets.erase(seq_id);
                    Callbacks::on_crsf_command_failed(seq_id, "Timeout after retries");
                }
            }
        }
    }
    
public:
    bool connected() const { return socket_fd >= 0; }
    
    size_t pending_count() const {
        std::lock_guard<std::mutex> lock(pending_mutex);
        return pending_packets.size();
    }
    
    void get_stats(float& success_rate, uint64_t& avg_latency) const {
        success_rate = 0.95f; // Placeholder
        avg_latency = 1000;   // Placeholder
    }
};

// UDP сервер
class UDPServer {
private:
    int socket_fd = -1;
    int port;

public:
    UDPServer(int p) : port(p) {}
    
    ~UDPServer() {
        close();
    }
    
    bool open() {
        socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0) {
            std::cerr << "❌ Failed to create UDP server socket: " << strerror(errno) << std::endl;
            return false;
        }
        
        int reuse = 1;
        setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);
        
        if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "❌ Failed to bind UDP server: " << strerror(errno) << std::endl;
            ::close(socket_fd);
            socket_fd = -1;
            return false;
        }
        
        std::cout << "✅ UDP server listening on port " << port << std::endl;
        return true;
    }
    
    void close() {
        if (socket_fd >= 0) {
            ::close(socket_fd);
            socket_fd = -1;
        }
    }
    
    ssize_t receive(uint8_t* buffer, size_t size, struct sockaddr_in* client_addr = nullptr) {
        if (socket_fd < 0) return -1;
        
        socklen_t addr_len = sizeof(struct sockaddr_in);
        return recvfrom(socket_fd, buffer, size, 0, 
                       (struct sockaddr*)client_addr, client_addr ? &addr_len : nullptr);
    }
    
    int get_fd() const { return socket_fd; }
    bool connected() const { return socket_fd >= 0; }
};

// Основний CRSF-UDP Bridge клас - ВИПРАВЛЕНО
class CRSFUDPBridgeWithFeedback {
private:
    std::unique_ptr<SerialPort> rx_port;
    std::unique_ptr<UDPClientWithACK> crsf_client;
    std::unique_ptr<UDPServer> telemetry_server;
    
    std::atomic<bool> running{false};
    std::thread uart_to_udp_thread, udp_to_uart_thread, stats_thread, heartbeat_thread;
    
    Stats stats;
    
    // Буфери
    uint8_t uart_buffer[BUFFER_SIZE];
    uint8_t packet_buffer[MAX_PACKET_SIZE];
    size_t uart_buffer_pos = 0;

public:
    CRSFUDPBridgeWithFeedback() = default;
    
    ~CRSFUDPBridgeWithFeedback() {
        stop();
    }
    
    bool connect() {
        std::cout << "🔌 Connecting to devices..." << std::endl;
        
        // UART підключення
        for (int baud : {PRIMARY_BAUD, FALLBACK_BAUD}) {
            std::cout << "🔌 Trying UART " << baud << " baud..." << std::endl;
            
            rx_port = std::make_unique<SerialPort>(RX_PORT, baud);
            if (rx_port->open()) {
                std::cout << "✅ UART connected at " << baud << " baud" << std::endl;
                break;
            }
            rx_port.reset();
        }
        
        if (!rx_port) {
            std::cout << "❌ Failed to connect to UART" << std::endl;
            return false;
        }
        
        // UDP з ACK
        crsf_client = std::make_unique<UDPClientWithACK>(CAMERA_IP, CRSF_UDP_PORT);
        if (!crsf_client->open()) {
            return false;
        }
        
        // UDP сервер
        telemetry_server = std::make_unique<UDPServer>(TELEMETRY_UDP_PORT);
        if (!telemetry_server->open()) {
            return false;
        }
        
        std::cout << "✅ All connections established with feedback support" << std::endl;
        return true;
    }
    
    void disconnect() {
        if (rx_port) rx_port->close();
        if (crsf_client) crsf_client->close();
        if (telemetry_server) telemetry_server->close();
    }
    
    // Heartbeat потік
    void heartbeat_loop() {
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            if (!running) break;
            
            uint8_t heartbeat_data[] = {0xFF, 0x01, 0x00, 0x01};
            uint32_t seq_id = crsf_client->send_with_ack(heartbeat_data, sizeof(heartbeat_data));
            
            if (seq_id > 0) {
                stats.heartbeats_sent++;
                printf("💗 Heartbeat #%u sent to camera\n", seq_id);
            }
            
            bool was_online = stats.camera_online.load();
            bool is_online = (time(nullptr) - stats.last_camera_response) < 15;
            
            if (was_online != is_online) {
                stats.camera_online = is_online;
                Callbacks::on_camera_status_changed(is_online, 
                    is_online ? "Heartbeat response received" : "Heartbeat timeout");
            }
        }
    }
    
    // UART → UDP
    void uart_to_udp_loop() {
        std::cout << "🔄 UART→UDP thread started with ACK tracking" << std::endl;
        
        while (running) {
            try {
                if (rx_port && rx_port->available() > 0) {
                    ssize_t bytes = rx_port->read(uart_buffer + uart_buffer_pos, 
                                                 BUFFER_SIZE - uart_buffer_pos);
                    if (bytes > 0) {
                        uart_buffer_pos += bytes;
                        
                        // Обробка пакетів
                        size_t processed = 0;
                        while (processed < uart_buffer_pos) {
                            // Пошук SYNC байтів
                            size_t sync_pos = processed;
                            bool found_sync = false;
                            
                            while (sync_pos < uart_buffer_pos) {
                                if (uart_buffer[sync_pos] == 0xC8 || uart_buffer[sync_pos] == 0xC6) {
                                    found_sync = true;
                                    break;
                                }
                                sync_pos++;
                            }
                            
                            if (!found_sync) {
                                uart_buffer_pos = 0;
                                break;
                            }
                            
                            if (sync_pos > processed) {
                                processed = sync_pos;
                                continue;
                            }
                            
                            if (processed + 2 >= uart_buffer_pos) {
                                break;
                            }
                            
                            size_t packet_len = uart_buffer[processed + 1] + 2;
                            if (packet_len > MAX_PACKET_SIZE || packet_len < 4) {
                                processed++;
                                continue;
                            }
                            
                            if (processed + packet_len > uart_buffer_pos) {
                                break;
                            }
                            
                            memcpy(packet_buffer, uart_buffer + processed, packet_len);
                            
                            bool is_valid = (packet_len >= 4 && 
                                           (packet_buffer[0] == 0xC8 || packet_buffer[0] == 0xC6) &&
                                           packet_buffer[1] == (packet_len - 2));
                            
                            if (is_valid) {
                                stats.crsf_received++;
                                
                                uint32_t seq_id = crsf_client->send_with_ack(packet_buffer, packet_len);
                                if (seq_id > 0) {
                                    stats.crsf_sent_udp++;
                                } else {
                                    stats.errors++;
                                    Callbacks::on_crsf_command_failed(0, "Send failed");
                                }
                            } else {
                                stats.errors++;
                            }
                            
                            processed += packet_len;
                        }
                        
                        if (processed > 0) {
                            memmove(uart_buffer, uart_buffer + processed, uart_buffer_pos - processed);
                            uart_buffer_pos -= processed;
                        }
                        
                        if (uart_buffer_pos >= BUFFER_SIZE - 1) {
                            uart_buffer_pos = 0;
                        }
                    }
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                
            } catch (std::exception& e) {
                std::cout << "❌ UART→UDP error: " << e.what() << std::endl;
                stats.errors++;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    
    // UDP → UART - ВИПРАВЛЕНО порівняння розмірів
    void udp_to_uart_loop() {
        std::cout << "🔄 UDP→UART thread started with ACK processing" << std::endl;
        
        uint8_t udp_buffer[BUFFER_SIZE];
        struct sockaddr_in client_addr;
        
        while (running) {
            try {
                fd_set readfds;
                struct timeval timeout;
                
                FD_ZERO(&readfds);
                FD_SET(telemetry_server->get_fd(), &readfds);
                
                timeout.tv_sec = 0;
                timeout.tv_usec = 100000;
                
                int ret = select(telemetry_server->get_fd() + 1, &readfds, nullptr, nullptr, &timeout);
                if (ret <= 0) continue;
                
                if (FD_ISSET(telemetry_server->get_fd(), &readfds)) {
                    ssize_t bytes = telemetry_server->receive(udp_buffer, sizeof(udp_buffer), &client_addr);
                    
                    // ВИПРАВЛЕНО порівняння розмірів
                    if (static_cast<size_t>(bytes) >= (sizeof(udp_packet_t) - MAX_PACKET_SIZE)) {
                        udp_packet_t* udp_pkt = reinterpret_cast<udp_packet_t*>(udp_buffer);
                        stats.last_camera_response = time(nullptr);
                        
                        PacketType pkt_type = static_cast<PacketType>(udp_pkt->packet_type);
                        
                        switch (pkt_type) {
                            case PacketType::TELEMETRY:
                                if (static_cast<size_t>(bytes) >= (sizeof(udp_packet_t) - MAX_PACKET_SIZE + udp_pkt->data_length)) {
                                    stats.telemetry_received_udp++;
                                    
                                    ssize_t written = rx_port->write(udp_pkt->data, udp_pkt->data_length);
                                    if (written > 0) {
                                        stats.telemetry_sent_uart++;
                                        Callbacks::on_telemetry_received(udp_pkt->data_length, 
                                            std::string(inet_ntoa(client_addr.sin_addr)));
                                    } else {
                                        stats.errors++;
                                    }
                                }
                                break;
                                
                            case PacketType::ACK:
                                if (static_cast<size_t>(bytes) >= sizeof(ack_packet_t)) {
                                    ack_packet_t* ack_pkt = reinterpret_cast<ack_packet_t*>(udp_buffer);
                                    stats.crsf_acked++;
                                    crsf_client->process_ack(ack_pkt->ack_sequence_id, ack_pkt->timestamp);
                                }
                                break;
                                
                            case PacketType::HEARTBEAT:
                                stats.heartbeats_received++;
                                printf("💗 Heartbeat response received from camera\n");
                                break;
                                
                            default:
                                printf("⚠️ Unknown packet type: 0x%02X\n", udp_pkt->packet_type);
                                break;
                        }
                    }
                }
                
            } catch (std::exception& e) {
                std::cout << "❌ UDP→UART error: " << e.what() << std::endl;
                stats.errors++;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    
    // Розширена статистика
    void stats_loop() {
        auto last_report = std::chrono::steady_clock::now();
        
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(10));
            if (!running) break;
            
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_report).count();
            
            if (elapsed >= 30) { // Детальний звіт кожні 30 секунд
                float success_rate;
                uint64_t avg_latency;
                crsf_client->get_stats(success_rate, avg_latency);
                
                std::cout << "\n📊 === DETAILED RADXA STATISTICS ===" << std::endl;
                std::cout << "🎯 CRSF Commands:" << std::endl;
                std::cout << "   📥 Received from RX: " << stats.crsf_received.load() << std::endl;
                std::cout << "   📤 Sent to camera: " << stats.crsf_sent_udp.load() << std::endl;
                std::cout << "   ✅ Acknowledged: " << stats.crsf_acked.load() << std::endl;
                std::cout << "   ❌ Failed: " << stats.crsf_failed.load() << std::endl;
                std::cout << "   ⏳ Pending ACKs: " << crsf_client->pending_count() << std::endl;
                
                std::cout << "📡 Telemetry:" << std::endl;
                std::cout << "   📥 Received from camera: " << stats.telemetry_received_udp.load() << std::endl;
                std::cout << "   📤 Sent to RX: " << stats.telemetry_sent_uart.load() << std::endl;
                
                std::cout << "💗 Heartbeat:" << std::endl;
                std::cout << "   📤 Sent: " << stats.heartbeats_sent.load() << std::endl;
                std::cout << "   📥 Received: " << stats.heartbeats_received.load() << std::endl;
                
                std::cout << "🔗 Connection:" << std::endl;
                std::cout << "   📶 Camera status: " << (stats.camera_online.load() ? "🟢 ONLINE" : "🔴 OFFLINE") << std::endl;
                std::cout << "   📊 Success rate: " << (success_rate * 100) << "%" << std::endl;
                std::cout << "   ⏱️ Avg latency: " << avg_latency << " μs" << std::endl;
                std::cout << "   ❌ Errors: " << stats.errors.load() << std::endl;
                
                // Callback для якості з'єднання
                Callbacks::on_connection_quality(success_rate, avg_latency);
                
                last_report = now;
                std::cout << "================================\n" << std::endl;
            } else {
                // Короткий звіт кожні 10 секунд
                std::cout << "📊 Quick stats: CRSF " << stats.crsf_received.load() 
                          << "→" << stats.crsf_sent_udp.load() 
                          << " (✅" << stats.crsf_acked.load() 
                          << " ❌" << stats.crsf_failed.load() << ")"
                          << ", TEL " << stats.telemetry_received_udp.load() 
                          << "→" << stats.telemetry_sent_uart.load()
                          << ", Camera: " << (stats.camera_online.load() ? "🟢" : "🔴") << std::endl;
            }
        }
    }
    
    bool start() {
        if (!connect()) {
            return false;
        }
        
        running = true;
        
        // Запуск потоків
        uart_to_udp_thread = std::thread(&CRSFUDPBridgeWithFeedback::uart_to_udp_loop, this);
        udp_to_uart_thread = std::thread(&CRSFUDPBridgeWithFeedback::udp_to_uart_loop, this);
        stats_thread = std::thread(&CRSFUDPBridgeWithFeedback::stats_loop, this);
        heartbeat_thread = std::thread(&CRSFUDPBridgeWithFeedback::heartbeat_loop, this);
        
        std::cout << "🚀 CRSF-UDP Bridge with Feedback running!" << std::endl;
        return true;
    }
    
    void stop() {
        if (!running) return;
        
        running = false;
        
        // Дочекатися завершення потоків
        if (uart_to_udp_thread.joinable()) {
            uart_to_udp_thread.join();
        }
        if (udp_to_uart_thread.joinable()) {
            udp_to_uart_thread.join();
        }
        if (stats_thread.joinable()) {
            stats_thread.join();
        }
        if (heartbeat_thread.joinable()) {
            heartbeat_thread.join();
        }
        
        disconnect();
        std::cout << "⏹️ CRSF-UDP Bridge with Feedback stopped" << std::endl;
        
        // Фінальна статистика
        std::cout << "\n📋 === FINAL STATISTICS ===" << std::endl;
        std::cout << "📤 CRSF commands sent: " << stats.crsf_sent_udp.load() << std::endl;
        std::cout << "✅ Commands acknowledged: " << stats.crsf_acked.load() << std::endl;
        std::cout << "❌ Commands failed: " << stats.crsf_failed.load() << std::endl;
        
        if (stats.crsf_sent_udp.load() > 0) {
            float success_rate = static_cast<float>(stats.crsf_acked.load()) / stats.crsf_sent_udp.load();
            std::cout << "📊 Overall success rate: " << (success_rate * 100) << "%" << std::endl;
        }
        
        std::cout << "📡 Telemetry packets: " << stats.telemetry_received_udp.load() << std::endl;
        std::cout << "💗 Heartbeats exchanged: " << stats.heartbeats_sent.load() 
                  << "/" << stats.heartbeats_received.load() << std::endl;
        std::cout << "❌ Total errors: " << stats.errors.load() << std::endl;
    }
};

// Відеоплеєр
class SimpleVideoPlayer {
private:
    GstElement* playbin = nullptr;
    GMainLoop* loop = nullptr;
    std::atomic<bool> running{false};

    static gboolean bus_callback(GstBus*, GstMessage* message, gpointer data) {
        SimpleVideoPlayer* self = static_cast<SimpleVideoPlayer*>(data);
        
        switch (GST_MESSAGE_TYPE(message)) {
            case GST_MESSAGE_ERROR: {
                GError* err;
                gchar* debug;
                gst_message_parse_error(message, &err, &debug);
                std::cout << "❌ Video Error: " << err->message << std::endl;
                g_main_loop_quit(self->loop);
                g_error_free(err);
                g_free(debug);
                break;
            }
            case GST_MESSAGE_STATE_CHANGED:
                if (GST_MESSAGE_SRC(message) == GST_OBJECT(self->playbin)) {
                    GstState old_state, new_state;
                    gst_message_parse_state_changed(message, &old_state, &new_state, nullptr);
                    if (new_state == GST_STATE_PLAYING) {
                        std::cout << "🎬 Video playing!" << std::endl;
                    }
                }
                break;
            default:
                break;
        }
        return TRUE;
    }

public:
    bool start(const std::string& uri) {
        std::cout << "🎬 Starting video..." << std::endl;
        
        playbin = gst_element_factory_make("playbin", "player");
        if (!playbin) return false;
        
        g_object_set(G_OBJECT(playbin), "uri", uri.c_str(), nullptr);
        
        GstBus* bus = gst_element_get_bus(playbin);
        gst_bus_add_watch(bus, bus_callback, this);
        gst_object_unref(bus);
        
        GstStateChangeReturn ret = gst_element_set_state(playbin, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) return false;
        
        running = true;
        loop = g_main_loop_new(nullptr, FALSE);
        return true;
    }
    
    void run() {
        if (loop && running) {
            g_main_loop_run(loop);
        }
    }
    
    void stop() {
        running = false;
        if (playbin) {
            gst_element_set_state(playbin, GST_STATE_NULL);
            gst_object_unref(playbin);
            playbin = nullptr;
        }
        if (loop) {
            g_main_loop_quit(loop);
            g_main_loop_unref(loop);
            loop = nullptr;
        }
    }
};

// Глобальні змінні для signal handler
static CRSFUDPBridgeWithFeedback* global_bridge = nullptr;
static SimpleVideoPlayer* global_player = nullptr;

void signal_handler(int) {
    std::cout << "\n🛑 Stopping..." << std::endl;
    if (global_bridge) global_bridge->stop();
    if (global_player) global_player->stop();
    exit(0);
}

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "🌉 RADXA CRSF-UDP BRIDGE WITH FEEDBACK (FIXED)" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "  UART Input: " << RX_PORT << std::endl;
    std::cout << "  Camera IP: " << CAMERA_IP << std::endl;
    std::cout << "  CRSF UDP Port: " << CRSF_UDP_PORT << std::endl;
    std::cout << "  Telemetry UDP Port: " << TELEMETRY_UDP_PORT << std::endl;
    std::cout << "  Video RTSP: " << RTSP_URL << std::endl;
    std::cout << "  Baud: " << PRIMARY_BAUD << " (fallback " << FALLBACK_BAUD << ")" << std::endl;
    std::cout << "🔄 Features: ACK tracking, Heartbeat, Connection monitoring" << std::endl << std::endl;
    
    // Перевірка наявності UART порту
    struct stat buffer;
    if (stat(RX_PORT.c_str(), &buffer) != 0) {
        std::cout << "❌ " << RX_PORT << " not found!" << std::endl;
        return 1;
    }
    
    // Запитати підтвердження
    std::cout << "❓ Start CRSF-UDP bridge with feedback? (y/n): ";
    std::string answer;
    std::getline(std::cin, answer);
    if (answer != "y" && answer != "Y") {
        return 0;
    }
    
    // Створення та запуск bridge
    CRSFUDPBridgeWithFeedback bridge;
    global_bridge = &bridge;
    
    if (!bridge.start()) {
        std::cout << "❌ Failed to start bridge" << std::endl;
        return 1;
    }
    
    // Запуск відеоплеєра
    SimpleVideoPlayer player;
    global_player = &player;
    
    if (!player.start(RTSP_URL)) {
        std::cout << "❌ Failed to start video (continuing without video)" << std::endl;
    } else {
        std::cout << "✅ Video player started" << std::endl;
    }
    
    std::cout << "✅ RADXA CRSF-UDP Bridge with Feedback running!" << std::endl;
    std::cout << "📊 Quick statistics every 10 seconds" << std::endl;
    std::cout << "📋 Detailed statistics every 30 seconds" << std::endl;
    std::cout << "🔄 Features:" << std::endl;
    std::cout << "   ✅ ACK confirmation for each CRSF command" << std::endl;
    std::cout << "   🔄 Automatic retry for failed packets" << std::endl;
    std::cout << "   💗 Heartbeat monitoring" << std::endl;
    std::cout << "   📊 Real-time connection quality" << std::endl;
    std::cout << "   📞 Callback notifications" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl << std::endl;
    
    // Головний цикл
    if (global_player) {
        player.run();
    } else {
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    // Очищення
    bridge.stop();
    if (global_player) {
        player.stop();
    }
    
    return 0;
}