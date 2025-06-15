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

// Конфігурація
const std::string RTSP_URL = "rtsp://root:12345@192.168.0.100:554/stream1";
const int CRSF_BAUD = 420000;
const std::string RX_PORT = "/dev/ttyUSB1";
const std::string FC_PORT = "/dev/ttyUSB0";

// CRSF Bridge (копія з вашого working коду)
class SimpleCRSFBridge {
private:
    std::string rx_port, fc_port;
    int baud_rate;
    int rx_fd = -1, fc_fd = -1;
    std::atomic<bool> running{false};
    std::atomic<long> rx_packets{0}, fc_packets{0};
    std::unique_ptr<std::thread> bridge_thread;

    bool setup_serial_port(int fd, int baud) {
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(fd, &tty) != 0) return false;
        
        // Базова швидкість для custom baud
        speed_t speed = B38400;
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        
        // 8N1 конфігурація
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        
        // Raw mode
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;
        
        if (tcsetattr(fd, TCSANOW, &tty) != 0) return false;
        
        // Custom baud rate
        if (baud != 38400) {
            struct serial_struct ss;
            if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
                ss.flags = (ss.flags & ~0x0030) | 0x0010;
                ss.custom_divisor = ss.baud_base / baud;
                if (ioctl(fd, TIOCSSERIAL, &ss) == 0) {
                    std::cout << "✅ Custom baud rate " << baud << " set successfully\n";
                } else {
                    std::cout << "⚠️ Custom baud failed, using 38400\n";
                }
            }
        }
        
        tcflush(fd, TCIOFLUSH);
        return true;
    }

    void bridge_loop() {
        char buffer[1024];
        while (running) {
            // RX → FC
            if (rx_fd >= 0) {
                ssize_t bytes = read(rx_fd, buffer, sizeof(buffer));
                if (bytes > 0) {
                    if (fc_fd >= 0) write(fc_fd, buffer, bytes);
                    rx_packets += bytes;
                }
            }
            
            // FC → RX
            if (fc_fd >= 0) {
                ssize_t bytes = read(fc_fd, buffer, sizeof(buffer));
                if (bytes > 0) {
                    if (rx_fd >= 0) write(rx_fd, buffer, bytes);
                    fc_packets += bytes;
                }
            }
            
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }

public:
    SimpleCRSFBridge(const std::string& rx, const std::string& fc, int baud)
        : rx_port(rx), fc_port(fc), baud_rate(baud) {}
    
    ~SimpleCRSFBridge() { stop(); }
    
    bool start() {
        std::cout << "🔌 Connecting CRSF bridge at " << baud_rate << " baud...\n";
        
        rx_fd = open(rx_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (rx_fd < 0) {
            std::cerr << "❌ Failed to open RX port: " << rx_port << "\n";
            return false;
        }
        
        fc_fd = open(fc_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fc_fd < 0) {
            std::cerr << "❌ Failed to open FC port: " << fc_port << "\n";
            close(rx_fd);
            return false;
        }
        
        if (!setup_serial_port(rx_fd, baud_rate) || !setup_serial_port(fc_fd, baud_rate)) {
            stop();
            return false;
        }
        
        running = true;
        bridge_thread = std::make_unique<std::thread>(&SimpleCRSFBridge::bridge_loop, this);
        
        std::cout << "✅ Bridge connected at " << baud_rate << " baud\n";
        std::cout << "🌉 Bridge: " << rx_port << " → " << fc_port << "\n";
        std::cout << "🚀 CRSF Bridge running!\n";
        return true;
    }
    
    void stop() {
        running = false;
        if (bridge_thread && bridge_thread->joinable()) {
            bridge_thread->join();
        }
        if (rx_fd >= 0) { close(rx_fd); rx_fd = -1; }
        if (fc_fd >= 0) { close(fc_fd); fc_fd = -1; }
        std::cout << "⏹️ Bridge stopped\n";
    }
    
    void print_stats() {
        std::cout << "\r🌉 Bridge: RX→FC: " << rx_packets.load() 
                  << " | FC→RX: " << fc_packets.load() << "    " << std::flush;
    }
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
                std::cout << "\n❌ Video Error: " << err->message << "\n";
                g_main_loop_quit(self->loop);
                g_error_free(err);
                g_free(debug);
                break;
            }
            case GST_MESSAGE_EOS:
                std::cout << "\n📺 End of stream\n";
                g_main_loop_quit(self->loop);
                break;
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
        std::cout << "🎬 Starting video player...\n";
        std::cout << "📡 URI: " << uri << "\n";
        
        // Використовуємо playbin - найпростіший елемент
        playbin = gst_element_factory_make("playbin", "player");
        if (!playbin) {
            std::cout << "❌ Failed to create playbin\n";
            return false;
        }
        
        // Встановити URI
        g_object_set(G_OBJECT(playbin), "uri", uri.c_str(), nullptr);
        
        // Встановити відеовихід (без аудіо)
        g_object_set(G_OBJECT(playbin), "flags", 0x00000001, nullptr); // GST_PLAY_FLAG_VIDEO
        
        // Message handler
        GstBus* bus = gst_element_get_bus(playbin);
        gst_bus_add_watch(bus, bus_callback, this);
        gst_object_unref(bus);
        
        // Запустити
        GstStateChangeReturn ret = gst_element_set_state(playbin, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            std::cout << "❌ Failed to start video playback\n";
            return false;
        }
        
        running = true;
        loop = g_main_loop_new(nullptr, FALSE);
        
        std::cout << "✅ Video player ready\n";
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

void signal_handler(int /*signal*/) {
    std::cout << "\n🛑 Stopping system...\n";
    if (global_bridge) global_bridge->stop();
    if (global_player) global_player->stop();
    exit(0);
}

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "🎬 ULTRA-SIMPLE CLEAN VIDEO + CRSF BRIDGE\n";
    std::cout << "==========================================\n";
    std::cout << "📡 RTSP: " << RTSP_URL << "\n";
    std::cout << "⚡ CRSF: " << CRSF_BAUD << " baud\n";
    std::cout << "🌉 Bridge: " << RX_PORT << " → " << FC_PORT << "\n\n";
    
    // Перевірити порти
    struct stat buffer;
    if (stat(RX_PORT.c_str(), &buffer) != 0) {
        std::cout << "❌ RX port " << RX_PORT << " not found!\n";
        return 1;
    }
    if (stat(FC_PORT.c_str(), &buffer) != 0) {
        std::cout << "❌ FC port " << FC_PORT << " not found!\n";
        return 1;
    }
    
    // Створити CRSF bridge
    SimpleCRSFBridge bridge(RX_PORT, FC_PORT, CRSF_BAUD);
    global_bridge = &bridge;
    
    if (!bridge.start()) {
        std::cout << "❌ Failed to start bridge\n";
        return 1;
    }
    
    // Створити відеоплеєр
    SimpleVideoPlayer player;
    global_player = &player;
    
    if (!player.start(RTSP_URL)) {
        std::cout << "❌ Failed to start video\n";
        bridge.stop();
        return 1;
    }
    
    // Запустити статус thread
    std::thread status_thread([&bridge]() {
        while (true) {
            bridge.print_stats();
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    });
    status_thread.detach();
    
    std::cout << "🚀 System running! Press Ctrl+C to stop\n";
    
    // Головний цикл
    player.run();
    
    // Зупинити все
    bridge.stop();
    player.stop();
    
    return 0;
}