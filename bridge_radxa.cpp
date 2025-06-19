#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>
#include <cstring>
#include <csignal>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

// GStreamer
#include <gst/gst.h>
#include <glib.h>

// Конфігурація - тільки 420000 baud
const std::string RTSP_URL = "rtsp://root:12345@192.168.0.100:554/stream1";
const std::string RX_PORT = "/dev/ttyUSB1";   // RX приймач
const std::string FC_PORT = "/dev/ttyUSB0";   // FC
const int CRSF_BAUD = 420000;  // Тільки цей baud

// Простий Serial Bridge (копія логіки Python)
class SimpleCppBridge {
private:
    int rx_fd = -1, fc_fd = -1;
    std::atomic<bool> running{false};
    std::atomic<long> rx_bytes{0}, fc_bytes{0}, errors{0};
    std::unique_ptr<std::thread> bridge_thread, stats_thread;

    bool setup_custom_baud_420000(int fd) {
        struct termios tty;
        
        if (tcgetattr(fd, &tty) != 0) {
            return false;
        }
        
        // Використовуємо B38400 як базу для custom baud
        cfsetispeed(&tty, B38400);
        cfsetospeed(&tty, B38400);
        
        // Базові налаштування 8N1
        tty.c_cflag &= ~PARENB;   // No parity
        tty.c_cflag &= ~CSTOPB;   // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;       // 8 bits
        tty.c_cflag |= CREAD | CLOCAL;
        
        // Raw mode
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;
        
        // Короткий timeout
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;
        
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            return false;
        }
        
        // Встановити custom baud 420000
        struct serial_struct ss;
        if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
            ss.flags = (ss.flags & ~0x0030) | 0x0010;  // ASYNC_SPD_CUST
            ss.custom_divisor = ss.baud_base / 420000;
            if (ioctl(fd, TIOCSSERIAL, &ss) == 0) {
                std::cout << "✅ Custom baud 420000 set successfully\n";
                return true;
            } else {
                std::cout << "❌ Failed to set custom baud 420000\n";
                return false;
            }
        }
        
        return false;
    }
    
    bool try_connect() {
        std::cout << "🔌 Connecting at " << CRSF_BAUD << " baud...\n";
        
        // Відкрити порти
        rx_fd = open(RX_PORT.c_str(), O_RDWR | O_NOCTTY);
        if (rx_fd < 0) {
            std::cout << "❌ Failed to open " << RX_PORT << "\n";
            return false;
        }
        
        fc_fd = open(FC_PORT.c_str(), O_RDWR | O_NOCTTY);
        if (fc_fd < 0) {
            std::cout << "❌ Failed to open " << FC_PORT << "\n";
            close(rx_fd);
            rx_fd = -1;
            return false;
        }
        
        // Налаштувати custom baud 420000
        if (!setup_custom_baud_420000(rx_fd) || !setup_custom_baud_420000(fc_fd)) {
            std::cout << "❌ Failed to configure 420000 baud\n";
            close(rx_fd);
            close(fc_fd);
            rx_fd = fc_fd = -1;
            return false;
        }
        
        std::cout << "✅ Connected at " << CRSF_BAUD << " baud\n";
        std::cout << "📡 Bridge: " << RX_PORT << " → " << FC_PORT << "\n";
        return true;
    }

    void bridge_loop() {
        std::cout << "🔄 Bridge thread started\n";
        
        char buffer[1024];
        
        while (running) {
            try {
                // USB1 → USB0 (RX → FC) - як у Python
                ssize_t rx_available = read(rx_fd, buffer, sizeof(buffer));
                if (rx_available > 0) {
                    write(fc_fd, buffer, rx_available);
                    rx_bytes += rx_available;
                }
                
                // USB0 → USB1 (FC → RX, телеметрія) - як у Python
                ssize_t fc_available = read(fc_fd, buffer, sizeof(buffer));
                if (fc_available > 0) {
                    write(rx_fd, buffer, fc_available);
                    fc_bytes += fc_available;
                }
                
                // 1ms sleep як у Python
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                
            } catch (...) {
                errors++;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    
    void stats_loop() {
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            std::cout << "📊 RX→FC: " << rx_bytes.load() << " bytes | "
                      << "FC→RX: " << fc_bytes.load() << " bytes | "
                      << "Errors: " << errors.load() << "\n";
        }
    }

public:
    bool connect() {
        // Тільки 420000 baud
        return try_connect();
    }
    
    bool start() {
        if (!connect()) {
            return false;
        }
        
        running = true;
        
        bridge_thread = std::make_unique<std::thread>(&SimpleCppBridge::bridge_loop, this);
        stats_thread = std::make_unique<std::thread>(&SimpleCppBridge::stats_loop, this);
        
        std::cout << "🚀 Bridge running!\n";
        return true;
    }
    
    void stop() {
        running = false;
        
        if (bridge_thread && bridge_thread->joinable()) {
            bridge_thread->join();
        }
        if (stats_thread && stats_thread->joinable()) {
            stats_thread->join();
        }
        
        if (rx_fd >= 0) { close(rx_fd); rx_fd = -1; }
        if (fc_fd >= 0) { close(fc_fd); fc_fd = -1; }
        
        std::cout << "⏹️ Bridge stopped\n";
    }
    
    int get_baud() const { return CRSF_BAUD; }
};


// Простий відеоплеєр
class SimpleVideoPlayer {
private:
    GstElement* playbin = nullptr;
    GMainLoop* loop = nullptr;
    std::atomic<bool> running{false};

    static gboolean bus_callback(GstBus* /*bus*/, GstMessage* message, gpointer data) {
        SimpleVideoPlayer* self = static_cast<SimpleVideoPlayer*>(data);
        
        switch (GST_MESSAGE_TYPE(message)) {
            case GST_MESSAGE_ERROR: {
                GError* err;
                gchar* debug;
                gst_message_parse_error(message, &err, &debug);
                std::cout << "❌ Video Error: " << err->message << "\n";
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
                        std::cout << "🎬 Video playing!\n";
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
        std::cout << "🎬 Starting video...\n";
        
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

// Глобальні змінні
static SimpleCppBridge* global_bridge = nullptr;
static SimpleVideoPlayer* global_player = nullptr;

void signal_handler(int /*signal*/) {
    std::cout << "\n🛑 Stopping...\n";
    if (global_bridge) global_bridge->stop();
    if (global_player) global_player->stop();
    exit(0);
}

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "🌉 SIMPLE C++ BRIDGE (Python Logic)\n";
    std::cout << "====================================\n";
    std::cout << "Configuration:\n";
    std::cout << "  RX Input:  " << RX_PORT << "\n";
    std::cout << "  FC Output: " << FC_PORT << "\n";
    std::cout << "  Direction: USB1 → USB0\n";
    std::cout << "  Baud: " << CRSF_BAUD << " (CRSF custom speed)\n";
    std::cout << "  Video: " << RTSP_URL << "\n\n";
    
    // Перевірити порти
    struct stat buffer;
    if (stat(RX_PORT.c_str(), &buffer) != 0) {
        std::cout << "❌ " << RX_PORT << " not found!\n";
        return 1;
    }
    if (stat(FC_PORT.c_str(), &buffer) != 0) {
        std::cout << "❌ " << FC_PORT << " not found!\n";
        return 1;
    }
    
    // Запустити bridge
    SimpleCppBridge bridge;
    global_bridge = &bridge;
    
    if (!bridge.start()) {
        std::cout << "❌ Failed to start bridge\n";
        return 1;
    }
    
    // Запустити відео
    SimpleVideoPlayer player;
    global_player = &player;
    
    if (!player.start(RTSP_URL)) {
        std::cout << "❌ Failed to start video\n";
        bridge.stop();
        return 1;
    }
    
    std::cout << "✅ System running at " << bridge.get_baud() << " baud!\n";
    std::cout << "📊 Statistics every 5 seconds\n";
    std::cout << "Press Ctrl+C to stop\n\n";
    
    // Головний цикл
    player.run();
    
    // Очищення
    bridge.stop();
    player.stop();
    
    return 0;
}