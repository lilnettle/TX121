#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>
#include <cstring>
#include <csignal>
#include <vector>
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

// GStreamer
#include <gst/gst.h>
#include <glib.h>

// НАЛАШТУВАННЯ
const std::string RX_PORT = "/dev/ttyUSB1";    // CRSF вхід від приймача
const std::string CAMERA_IP = "192.168.0.100"; // IP камери
const std::string RTSP_URL = "rtsp://root:12345@192.168.0.100:554/stream1";
const int CRSF_UDP_PORT = 5000;               // Порт для відправки CRSF на камеру
const int TELEMETRY_UDP_PORT = 5001;          // Порт для отримання телеметрії з камери
const int PRIMARY_BAUD = 420000;
const int FALLBACK_BAUD = 115200;

// Константи протоколу
const uint8_t CRSF_SYNC_BYTE = 0xC8;
const size_t MAX_PACKET_SIZE = 256;
const size_t BUFFER_SIZE = 1024;

// UDP пакет структура (сумісна з камерою)
typedef struct {
    uint64_t timestamp;
    uint8_t packet_type;    // 0x01 - CRSF команди, 0x02 - телеметрія
    uint16_t data_length;
    uint8_t data[MAX_PACKET_SIZE];
} __attribute__((packed)) udp_packet_t;

// Статистика
struct Stats {
    std::atomic<uint32_t> crsf_received{0};
    std::atomic<uint32_t> crsf_sent_udp{0};
    std::atomic<uint32_t> telemetry_received_udp{0};
    std::atomic<uint32_t> telemetry_sent_uart{0};
    std::atomic<uint32_t> errors{0};
    time_t start_time;
    
    Stats() { start_time = time(nullptr); }
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
        
        // Сирий режим
        cfmakeraw(&tty);
        
        // Timeouts
        tty.c_cc[VMIN] = 0;   // Non-blocking
        tty.c_cc[VTIME] = 1;  // 100ms timeout
        
        // Додаткові налаштування для стабільності
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
        
        // Встановлення швидкості
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
        
        // Для кастомних швидкостей
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

// UDP клієнт для відправки CRSF на камеру
class UDPClient {
private:
    int socket_fd = -1;
    struct sockaddr_in server_addr;
    std::string server_ip;
    int server_port;

public:
    UDPClient(const std::string& ip, int port) : server_ip(ip), server_port(port) {}
    
    ~UDPClient() {
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
        
        std::cout << "✅ UDP client configured for " << server_ip << ":" << server_port << std::endl;
        return true;
    }
    
    void close() {
        if (socket_fd >= 0) {
            ::close(socket_fd);
            socket_fd = -1;
        }
    }
    
    ssize_t send(const uint8_t* data, size_t size) {
        if (socket_fd < 0) return -1;
        
        return sendto(socket_fd, data, size, 0, 
                     (struct sockaddr*)&server_addr, sizeof(server_addr));
    }
    
    bool connected() const { return socket_fd >= 0; }
};

// UDP сервер для отримання телеметрії з камери
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

// CRSF протокол функції
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

bool validate_crsf_packet(const uint8_t* packet, size_t len) {
    if (len < 4) return false;
    if (packet[0] != CRSF_SYNC_BYTE) return false;
    
    size_t expected_len = packet[1] + 2;
    if (len != expected_len) return false;
    
    uint8_t crc = 0;
    for (size_t i = 2; i < len - 1; i++) {
        crc = crc8_dvb_s2(crc, packet[i]);
    }
    
    return crc == packet[len - 1];
}

// Основний CRSF-UDP Bridge клас
class CRSFUDPBridge {
private:
    std::unique_ptr<SerialPort> rx_port;
    std::unique_ptr<UDPClient> crsf_client;
    std::unique_ptr<UDPServer> telemetry_server;
    
    std::atomic<bool> running{false};
    std::thread uart_to_udp_thread, udp_to_uart_thread, stats_thread;
    
    Stats stats;
    
    // Буфери
    uint8_t uart_buffer[BUFFER_SIZE];
    uint8_t packet_buffer[MAX_PACKET_SIZE];
    size_t uart_buffer_pos = 0;

public:
    CRSFUDPBridge() = default;
    
    ~CRSFUDPBridge() {
        stop();
    }
    
    bool connect() {
        std::cout << "🔌 Connecting to devices..." << std::endl;
        
        // Спроба підключення до UART з різними швидкостями
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
        
        // Підключення UDP клієнта для CRSF
        crsf_client = std::make_unique<UDPClient>(CAMERA_IP, CRSF_UDP_PORT);
        if (!crsf_client->open()) {
            return false;
        }
        
        // Підключення UDP сервера для телеметрії
        telemetry_server = std::make_unique<UDPServer>(TELEMETRY_UDP_PORT);
        if (!telemetry_server->open()) {
            return false;
        }
        
        std::cout << "✅ All connections established" << std::endl;
        std::cout << "📡 UART " << RX_PORT << " ↔ UDP " << CAMERA_IP << std::endl;
        return true;
    }
    
    void disconnect() {
        if (rx_port) rx_port->close();
        if (crsf_client) crsf_client->close();
        if (telemetry_server) telemetry_server->close();
    }
    
    // Потік: UART → UDP (CRSF команди на камеру)
    void uart_to_udp_loop() {
        std::cout << "🔄 UART→UDP thread started" << std::endl;
        
        while (running) {
            try {
                // Читання з UART
                if (rx_port && rx_port->available() > 0) {
                    ssize_t bytes = rx_port->read(uart_buffer + uart_buffer_pos, 
                                                 BUFFER_SIZE - uart_buffer_pos);
                    if (bytes > 0) {
                        uart_buffer_pos += bytes;
                        
                        // Обробка пакетів
                        size_t processed = 0;
                        while (processed < uart_buffer_pos) {
                            // Пошук SYNC байту
                            size_t sync_pos = processed;
                            while (sync_pos < uart_buffer_pos && uart_buffer[sync_pos] != CRSF_SYNC_BYTE) {
                                sync_pos++;
                            }
                            
                            if (sync_pos >= uart_buffer_pos) {
                                uart_buffer_pos = 0;
                                break;
                            }
                            
                            if (sync_pos > processed) {
                                processed = sync_pos;
                                continue;
                            }
                            
                            // Перевірка довжини пакету
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
                            
                            // Копіювання та валідація пакету
                            memcpy(packet_buffer, uart_buffer + processed, packet_len);
                            
                            if (validate_crsf_packet(packet_buffer, packet_len)) {
                                stats.crsf_received++;
                                
                                // Створення UDP пакету
                                udp_packet_t udp_pkt;
                                struct timespec ts;
                                clock_gettime(CLOCK_REALTIME, &ts);
                                udp_pkt.timestamp = ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
                                udp_pkt.packet_type = 0x01; // CRSF команди
                                udp_pkt.data_length = packet_len;
                                memcpy(udp_pkt.data, packet_buffer, packet_len);
                                
                                // Відправка на камеру
                                ssize_t sent = crsf_client->send((uint8_t*)&udp_pkt, 11 + packet_len);
                                if (sent > 0) {
                                    stats.crsf_sent_udp++;
                                    std::cout << "📡 UART→UDP: " << packet_len << " bytes CRSF" << std::endl;
                                } else {
                                    stats.errors++;
                                }
                            } else {
                                stats.errors++;
                            }
                            
                            processed += packet_len;
                        }
                        
                        // Компактування буфера
                        if (processed > 0) {
                            memmove(uart_buffer, uart_buffer + processed, uart_buffer_pos - processed);
                            uart_buffer_pos -= processed;
                        }
                        
                        // Захист від переповнення
                        if (uart_buffer_pos >= BUFFER_SIZE - 1) {
                            std::cout << "⚠️ UART buffer overflow protection" << std::endl;
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
        
        std::cout << "🔄 UART→UDP thread stopped" << std::endl;
    }
    
    // Потік: UDP → UART (телеметрія з камери)
    void udp_to_uart_loop() {
        std::cout << "🔄 UDP→UART thread started" << std::endl;
        
        uint8_t udp_buffer[BUFFER_SIZE];
        struct sockaddr_in client_addr;
        
        while (running) {
            try {
                // Використання select для неблокуючого читання
                fd_set readfds;
                struct timeval timeout;
                
                FD_ZERO(&readfds);
                FD_SET(telemetry_server->get_fd(), &readfds);
                
                timeout.tv_sec = 0;
                timeout.tv_usec = 100000; // 100ms
                
                int ret = select(telemetry_server->get_fd() + 1, &readfds, nullptr, nullptr, &timeout);
                if (ret <= 0) continue;
                
                if (FD_ISSET(telemetry_server->get_fd(), &readfds)) {
                    ssize_t bytes = telemetry_server->receive(udp_buffer, sizeof(udp_buffer), &client_addr);
                    
                    if (bytes >= 11) { // Мінімальний розмір UDP пакету
                        udp_packet_t* udp_pkt = (udp_packet_t*)udp_buffer;
                        
                        if (udp_pkt->packet_type == 0x02 && // Телеметрія
                            udp_pkt->data_length <= MAX_PACKET_SIZE &&
                            bytes >= 11 + udp_pkt->data_length) {
                            
                            stats.telemetry_received_udp++;
                            
                            // Відправка в UART
                            ssize_t written = rx_port->write(udp_pkt->data, udp_pkt->data_length);
                            if (written > 0) {
                                stats.telemetry_sent_uart++;
                                std::cout << "📡 UDP→UART: " << udp_pkt->data_length << " bytes telemetry" << std::endl;
                            } else {
                                stats.errors++;
                                std::cout << "❌ Write to UART failed" << std::endl;
                            }
                        }
                    }
                }
                
            } catch (std::exception& e) {
                std::cout << "❌ UDP→UART error: " << e.what() << std::endl;
                stats.errors++;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        std::cout << "🔄 UDP→UART thread stopped" << std::endl;
    }
    
    // Потік статистики
    void stats_loop() {
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(10));
            if (running) {
                std::cout << "📊 CRSF RX: " << stats.crsf_received.load() 
                          << ", UDP TX: " << stats.crsf_sent_udp.load()
                          << ", TEL RX: " << stats.telemetry_received_udp.load()
                          << ", UART TX: " << stats.telemetry_sent_uart.load()
                          << ", Errors: " << stats.errors.load() << std::endl;
            }
        }
    }
    
    bool start() {
        if (!connect()) {
            return false;
        }
        
        running = true;
        
        // Запуск потоків
        uart_to_udp_thread = std::thread(&CRSFUDPBridge::uart_to_udp_loop, this);
        udp_to_uart_thread = std::thread(&CRSFUDPBridge::udp_to_uart_loop, this);
        stats_thread = std::thread(&CRSFUDPBridge::stats_loop, this);
        
        std::cout << "🚀 CRSF-UDP Bridge running!" << std::endl;
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
        
        disconnect();
        std::cout << "⏹️ CRSF-UDP Bridge stopped" << std::endl;
        
        // Фінальна статистика
        std::cout << "📋 Final Statistics:" << std::endl;
        std::cout << "   CRSF received from UART: " << stats.crsf_received.load() << std::endl;
        std::cout << "   CRSF sent to camera UDP: " << stats.crsf_sent_udp.load() << std::endl;
        std::cout << "   Telemetry received from camera UDP: " << stats.telemetry_received_udp.load() << std::endl;
        std::cout << "   Telemetry sent to UART: " << stats.telemetry_sent_uart.load() << std::endl;
        std::cout << "   Total errors: " << stats.errors.load() << std::endl;
    }
};

// Відеоплеєр (без змін)
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
static CRSFUDPBridge* global_bridge = nullptr;
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
    
    std::cout << "🌉 CRSF-UDP BRIDGE FOR CAMERA" << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "  UART Input: " << RX_PORT << std::endl;
    std::cout << "  Camera IP: " << CAMERA_IP << std::endl;
    std::cout << "  CRSF UDP Port: " << CRSF_UDP_PORT << std::endl;
    std::cout << "  Telemetry UDP Port: " << TELEMETRY_UDP_PORT << std::endl;
    std::cout << "  Video RTSP: " << RTSP_URL << std::endl;
    std::cout << "  Baud: " << PRIMARY_BAUD << " (fallback " << FALLBACK_BAUD << ")" << std::endl << std::endl;
    
    // Перевірка наявності UART порту
    struct stat buffer;
    if (stat(RX_PORT.c_str(), &buffer) != 0) {
        std::cout << "❌ " << RX_PORT << " not found!" << std::endl;
        return 1;
    }
    
    // Запитати підтвердження
    std::cout << "❓ Start CRSF-UDP bridge? (y/n): ";
    std::string answer;
    std::getline(std::cin, answer);
    if (answer != "y" && answer != "Y") {
        return 0;
    }
    
    // Створення та запуск bridge
    CRSFUDPBridge bridge;
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
        // Не виходимо, bridge може працювати без відео
    } else {
        std::cout << "✅ Video player started" << std::endl;
    }
    
    std::cout << "✅ CRSF-UDP Bridge running!" << std::endl;
    std::cout << "📊 Statistics every 10 seconds" << std::endl;
    std::cout << "📡 UART→UDP: CRSF commands to camera" << std::endl;
    std::cout << "📡 UDP→UART: Telemetry from camera" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl << std::endl;
    
    // Головний цикл
    if (global_player) {
        // Якщо відео запущено, використовуємо GStreamer main loop
        player.run();
    } else {
        // Якщо відео не запущено, просто чекаємо
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