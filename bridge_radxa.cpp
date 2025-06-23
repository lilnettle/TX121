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

// GStreamer
#include <gst/gst.h>
#include <glib.h>

// Конфігурація
const std::string RTSP_URL = "rtsp://root:12345@192.168.0.100:554/stream1";
const std::string RX_PORT = "/dev/ttyUSB1";   // RX приймач
const std::string FC_PORT = "/dev/ttyUSB0";   // FC
const int PRIMARY_BAUD = 420000;
const int FALLBACK_BAUD = 115200;

// Простий серійний порт клас (як у Python)
class SimpleSerial {
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
        
        // Спроба встановити стандартну швидкість
        speed_t speed;
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
                // Для кастомних швидкостей використовуємо 38400 як базу
                speed = B38400;
                break;
        }
        
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        
        if (tcsetattr(fd, TCSANOW, &tty) < 0) {
            std::cerr << "❌ tcsetattr failed for " << port_name << std::endl;
            return false;
        }
        
        // Для кастомних швидкостей
        if (baud == 420000) {
            struct serial_struct ss;
            if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
                ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
                ss.custom_divisor = ss.baud_base / baud;
                
                if (ioctl(fd, TIOCSSERIAL, &ss) == 0) {
                    std::cout << "✅ " << port_name << ": custom baud " << baud << " set" << std::endl;
                } else {
                    std::cout << "⚠️ " << port_name << ": failed to set custom baud, using " << 38400 << std::endl;
                }
            }
        }
        
        return true;
    }

public:
    SimpleSerial(const std::string& port, int baud) 
        : port_name(port), baud_rate(baud) {}
    
    ~SimpleSerial() {
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
        
        // Очистити буфери
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
            // Реальна помилка (не просто відсутність даних)
            return -1;
        }
        return result > 0 ? result : 0;
    }
    
    ssize_t write(const uint8_t* buffer, size_t size) {
        if (!is_open) return -1;
        
        ssize_t result = ::write(fd, buffer, size);
        if (result > 0) {
            fsync(fd);  // Переконатися що дані записані
        }
        return result;
    }
    
    bool connected() const { return is_open; }
    const std::string& get_name() const { return port_name; }
    int get_baud() const { return baud_rate; }
};

// Простий CRSF Bridge (як у Python)
class SimpleCRSFBridge {
private:
    std::unique_ptr<SimpleSerial> rx_serial, fc_serial;
    std::atomic<bool> running{false};
    std::thread bridge_thread, stats_thread;
    
    // Статистика
    std::atomic<long> rx_packets{0}, fc_packets{0}, errors{0};
    
    // Буфери
    static constexpr size_t BUFFER_SIZE = 1024;
    uint8_t rx_buffer[BUFFER_SIZE];
    uint8_t fc_buffer[BUFFER_SIZE];

public:
    SimpleCRSFBridge() = default;
    
    ~SimpleCRSFBridge() {
        stop();
    }
    
    bool connect() {
        std::cout << "🔌 Trying to connect..." << std::endl;
        
        // Спроба підключення з різними швидкостями
        for (int baud : {PRIMARY_BAUD, FALLBACK_BAUD}) {
            std::cout << "🔌 Trying " << baud << " baud..." << std::endl;
            
            try {
                rx_serial = std::make_unique<SimpleSerial>(RX_PORT, baud);
                fc_serial = std::make_unique<SimpleSerial>(FC_PORT, baud);
                
                if (rx_serial->open() && fc_serial->open()) {
                    std::cout << "✅ Connected at " << baud << " baud" << std::endl;
                    std::cout << "📡 Bridge: " << RX_PORT << " → " << FC_PORT << std::endl;
                    return true;
                }
                
                // Закрити якщо не вдалося
                if (rx_serial) rx_serial->close();
                if (fc_serial) fc_serial->close();
                
            } catch (std::exception& e) {
                std::cout << "❌ Failed at " << baud << ": " << e.what() << std::endl;
            }
        }
        
        std::cout << "❌ All connection attempts failed" << std::endl;
        return false;
    }
    
    void disconnect() {
        if (rx_serial) rx_serial->close();
        if (fc_serial) fc_serial->close();
    }
    
    void bridge_loop() {
        std::cout << "🔄 Bridge thread started" << std::endl;
        
        while (running) {
            try {
                bool activity = false;
                
                // USB1 → USB0 (RX → FC)
                if (rx_serial && rx_serial->available() > 0) {
                    ssize_t bytes_read = rx_serial->read(rx_buffer, BUFFER_SIZE);
                    if (bytes_read > 0) {
                        ssize_t bytes_written = fc_serial->write(rx_buffer, bytes_read);
                        if (bytes_written > 0) {
                            rx_packets += bytes_written;
                            activity = true;
                        }
                    } else if (bytes_read < 0) {
                        std::cout << "❌ RX read error" << std::endl;
                        errors++;
                    }
                }
                
                // USB0 → USB1 (FC → RX, телеметрія)
                if (fc_serial && fc_serial->available() > 0) {
                    ssize_t bytes_read = fc_serial->read(fc_buffer, BUFFER_SIZE);
                    if (bytes_read > 0) {
                        ssize_t bytes_written = rx_serial->write(fc_buffer, bytes_read);
                        if (bytes_written > 0) {
                            fc_packets += bytes_written;
                            activity = true;
                        }
                    } else if (bytes_read < 0) {
                        std::cout << "❌ FC read error" << std::endl;
                        errors++;
                    }
                }
                
                // Затримка (як у Python)
                if (!activity) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                
            } catch (std::exception& e) {
                std::cout << "❌ Bridge error: " << e.what() << std::endl;
                errors++;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        std::cout << "🔄 Bridge thread stopped" << std::endl;
    }
    
    void stats_loop() {
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            if (running) {
                std::cout << "📊 RX→FC: " << rx_packets.load() << " bytes | "
                          << "FC→RX: " << fc_packets.load() << " bytes | "
                          << "Errors: " << errors.load() << std::endl;
            }
        }
    }
    
    bool start() {
        if (!connect()) {
            return false;
        }
        
        running = true;
        
        // Запустити потоки
        bridge_thread = std::thread(&SimpleCRSFBridge::bridge_loop, this);
        stats_thread = std::thread(&SimpleCRSFBridge::stats_loop, this);
        
        std::cout << "🚀 Bridge running!" << std::endl;
        return true;
    }
    
    void stop() {
        if (!running) return;
        
        running = false;
        
        // Дочекатися завершення потоків
        if (bridge_thread.joinable()) {
            bridge_thread.join();
        }
        if (stats_thread.joinable()) {
            stats_thread.join();
        }
        
        disconnect();
        std::cout << "⏹️ Bridge stopped" << std::endl;
        
        // Фінальна статистика
        std::cout << "📋 Final stats:" << std::endl;
        std::cout << "   RX→FC: " << rx_packets.load() << " bytes" << std::endl;
        std::cout << "   FC→RX: " << fc_packets.load() << " bytes" << std::endl;
        std::cout << "   Errors: " << errors.load() << std::endl;
    }
};

// Простий відеоплеєр
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
static SimpleCRSFBridge* global_bridge = nullptr;
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
    
    std::cout << "🌉 QUICK USB1→USB0 CRSF BRIDGE (C++)" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "  RX Input:  " << RX_PORT << std::endl;
    std::cout << "  FC Output: " << FC_PORT << std::endl;
    std::cout << "  Direction: USB1 → USB0" << std::endl;
    std::cout << "  Baud: " << PRIMARY_BAUD << " (fallback to " << FALLBACK_BAUD << ")" << std::endl;
    std::cout << "  Video: " << RTSP_URL << std::endl << std::endl;
    
    // Перевірка наявності портів
    struct stat buffer;
    if (stat(RX_PORT.c_str(), &buffer) != 0) {
        std::cout << "❌ " << RX_PORT << " not found!" << std::endl;
        return 1;
    }
    if (stat(FC_PORT.c_str(), &buffer) != 0) {
        std::cout << "❌ " << FC_PORT << " not found!" << std::endl;
        return 1;
    }
    
    // Запитати підтвердження
    std::cout << "❓ Start bridge? (y/n): ";
    std::string answer;
    std::getline(std::cin, answer);
    if (answer != "y" && answer != "Y") {
        return 0;
    }
    
    // Створення та запуск bridge
    SimpleCRSFBridge bridge;
    global_bridge = &bridge;
    
    if (!bridge.start()) {
        std::cout << "❌ Failed to start bridge" << std::endl;
        return 1;
    }
    
    // Запуск відеоплеєра
    SimpleVideoPlayer player;
    global_player = &player;
    
    if (!player.start(RTSP_URL)) {
        std::cout << "❌ Failed to start video" << std::endl;
        bridge.stop();
        return 1;
    }
    
    std::cout << "✅ Bridge running! Press Ctrl+C to stop" << std::endl;
    std::cout << "📊 Statistics every 5 seconds" << std::endl << std::endl;
    
    // Головний цикл (GStreamer)
    player.run();
    
    // Очищення
    bridge.stop();
    player.stop();
    
    return 0;
}