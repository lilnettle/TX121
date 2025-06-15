#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>
#include <cstring>
#include <csignal>
#include <cerrno>
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

// Оптимізований CRSF Bridge
class OptimizedCRSFBridge {
private:
    std::string rx_port, fc_port;
    int baud_rate;
    int rx_fd = -1, fc_fd = -1;
    std::atomic<bool> running{false};
    std::atomic<long> rx_packets{0}, fc_packets{0}, rx_bytes{0}, fc_bytes{0};
    std::unique_ptr<std::thread> bridge_thread;

    bool setup_serial_port(int fd, int baud) {
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(fd, &tty) != 0) return false;
        
        // Custom baud base
        speed_t speed = B38400;
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        
        // 8N1, no flow control
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
        
        // Minimal timeout for ultra-low latency
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;  // No timeout
        
        if (tcsetattr(fd, TCSANOW, &tty) != 0) return false;
        
        // Custom baud rate setup
        if (baud != 38400) {
            struct serial_struct ss;
            if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
                ss.flags = (ss.flags & ~0x0030) | 0x0010;
                ss.custom_divisor = ss.baud_base / baud;
                ioctl(fd, TIOCSSERIAL, &ss);
            }
        }
        
        tcflush(fd, TCIOFLUSH);
        return true;
    }

    void bridge_loop() {
        char rx_buffer[2048], fc_buffer[2048];
        
        // Set thread priority for real-time performance
        struct sched_param param;
        param.sched_priority = 50;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        
        while (running) {
            bool activity = false;
            
            // RX → FC (Commands - highest priority)
            if (rx_fd >= 0) {
                ssize_t bytes = read(rx_fd, rx_buffer, sizeof(rx_buffer));
                if (bytes > 0) {
                    if (fc_fd >= 0) {
                        write(fc_fd, rx_buffer, bytes);
                    }
                    rx_packets++;
                    rx_bytes += bytes;
                    activity = true;
                }
            }
            
            // FC → RX (Telemetry)
            if (fc_fd >= 0) {
                ssize_t bytes = read(fc_fd, fc_buffer, sizeof(fc_buffer));
                if (bytes > 0) {
                    if (rx_fd >= 0) {
                        write(rx_fd, fc_buffer, bytes);
                    }
                    fc_packets++;
                    fc_bytes += bytes;
                    activity = true;
                }
            }
            
            // Ultra-low latency sleep
            if (!activity) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
    }

public:
    OptimizedCRSFBridge(const std::string& rx, const std::string& fc, int baud)
        : rx_port(rx), fc_port(fc), baud_rate(baud) {}
    
    ~OptimizedCRSFBridge() { stop(); }
    
    bool start() {
        std::cout << "🔌 Starting optimized CRSF bridge at " << baud_rate << " baud...\n";
        
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
        bridge_thread = std::make_unique<std::thread>(&OptimizedCRSFBridge::bridge_loop, this);
        
        std::cout << "✅ Optimized bridge running\n";
        std::cout << "🌉 " << rx_port << " ↔ " << fc_port << " @ " << baud_rate << " baud\n";
        return true;
    }
    
    void stop() {
        running = false;
        if (bridge_thread && bridge_thread->joinable()) {
            bridge_thread->join();
        }
        if (rx_fd >= 0) { close(rx_fd); rx_fd = -1; }
        if (fc_fd >= 0) { close(fc_fd); fc_fd = -1; }
    }
    
    void print_stats() {
        std::cout << "\r🌉 Bridge: Commands: " << rx_packets.load() << " (" << rx_bytes.load() << "B)"
                  << " | Telemetry: " << fc_packets.load() << " (" << fc_bytes.load() << "B)    " << std::flush;
    }
};

// Оптимізований відеоплеєр
class LowLatencyVideoPlayer {
private:
    GstElement* playbin = nullptr;
    GMainLoop* loop = nullptr;
    std::atomic<bool> running{false};

    static gboolean bus_callback(GstBus* /*bus*/, GstMessage* message, gpointer data) {
        LowLatencyVideoPlayer* self = static_cast<LowLatencyVideoPlayer*>(data);
        
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
                std::cout << "\n📺 Video stream ended\n";
                g_main_loop_quit(self->loop);
                break;
            case GST_MESSAGE_STATE_CHANGED:
                if (GST_MESSAGE_SRC(message) == GST_OBJECT(self->playbin)) {
                    GstState old_state, new_state;
                    gst_message_parse_state_changed(message, &old_state, &new_state, nullptr);
                    if (new_state == GST_STATE_PLAYING) {
                        std::cout << "🎬 Low-latency video playing!\n";
                    }
                }
                break;
            case GST_MESSAGE_BUFFERING: {
                gint percent;
                gst_message_parse_buffering(message, &percent);
                // Force play even with low buffer for minimal latency
                if (percent >= 10) {
                    gst_element_set_state(self->playbin, GST_STATE_PLAYING);
                }
                break;
            }
            default:
                break;
        }
        return TRUE;
    }

public:
    bool start(const std::string& uri) {
        std::cout << "🎬 Starting low-latency video player...\n";
        std::cout << "📡 Stream: " << uri << "\n";
        
        // Set environment for minimal latency
        g_setenv("XDG_RUNTIME_DIR", "/tmp", TRUE);
        
        playbin = gst_element_factory_make("playbin", "player");
        if (!playbin) {
            std::cout << "❌ Failed to create playbin\n";
            return false;
        }
        
        // Configure for ultra-low latency
        g_object_set(G_OBJECT(playbin), 
            "uri", uri.c_str(),
            "flags", 0x00000001,  // Video only
            "buffer-size", 512,   // Minimal buffer
            "buffer-duration", (gint64)50000000,  // 50ms max
            nullptr);
        
        // Configure RTSP source for low latency
        g_signal_connect(playbin, "source-setup", G_CALLBACK(+[](GstElement* /*playbin*/, GstElement* source, gpointer /*data*/) {
            if (source && GST_IS_ELEMENT(source)) {
                const gchar* factory_name = gst_plugin_feature_get_name(GST_PLUGIN_FEATURE(gst_element_get_factory(source)));
                if (g_str_has_prefix(factory_name, "rtspsrc")) {
                    g_object_set(source,
                        "latency", 0,
                        "drop-on-latency", TRUE,
                        "do-retransmission", FALSE,
                        "timeout", (guint64)2000000,  // 2 sec
                        "tcp-timeout", (guint64)1000000,  // 1 sec
                        nullptr);
                }
            }
        }), nullptr);
        
        // Message handler
        GstBus* bus = gst_element_get_bus(playbin);
        gst_bus_add_watch(bus, bus_callback, this);
        gst_object_unref(bus);
        
        // Start playing
        GstStateChangeReturn ret = gst_element_set_state(playbin, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            std::cout << "❌ Failed to start video\n";
            return false;
        }
        
        running = true;
        loop = g_main_loop_new(nullptr, FALSE);
        
        std::cout << "✅ Low-latency video ready\n";
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
static OptimizedCRSFBridge* global_bridge = nullptr;
static LowLatencyVideoPlayer* global_player = nullptr;

void signal_handler(int /*signal*/) {
    std::cout << "\n🛑 Stopping optimized system...\n";
    if (global_bridge) global_bridge->stop();
    if (global_player) global_player->stop();
    exit(0);
}

int main(int argc, char* argv[]) {
    // Initialize
    gst_init(&argc, &argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Set process priority
    nice(-10);
    
    std::cout << "🚀 OPTIMIZED CLEAN VIDEO + CRSF BRIDGE\n";
    std::cout << "======================================\n";
    std::cout << "📡 RTSP: " << RTSP_URL << "\n";
    std::cout << "⚡ CRSF: " << CRSF_BAUD << " baud (optimized)\n";
    std::cout << "🌉 Bridge: " << RX_PORT << " ↔ " << FC_PORT << "\n";
    std::cout << "🎯 Mode: Ultra-low latency\n\n";
    
    // Check ports
    struct stat buffer;
    if (stat(RX_PORT.c_str(), &buffer) != 0) {
        std::cout << "❌ RX port " << RX_PORT << " not found!\n";
        return 1;
    }
    if (stat(FC_PORT.c_str(), &buffer) != 0) {
        std::cout << "❌ FC port " << FC_PORT << " not found!\n";
        return 1;
    }
    
    // Start optimized bridge
    OptimizedCRSFBridge bridge(RX_PORT, FC_PORT, CRSF_BAUD);
    global_bridge = &bridge;
    
    if (!bridge.start()) {
        std::cout << "❌ Failed to start bridge\n";
        return 1;
    }
    
    // Start low-latency video
    LowLatencyVideoPlayer player;
    global_player = &player;
    
    if (!player.start(RTSP_URL)) {
        std::cout << "❌ Failed to start video\n";
        bridge.stop();
        return 1;
    }
    
    // Status thread
    std::thread status_thread([&bridge]() {
        while (true) {
            bridge.print_stats();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    status_thread.detach();
    
    std::cout << "\n🎮 System ready! Enjoy low-latency FPV!\n";
    std::cout << "Press Ctrl+C to stop\n\n";
    
    // Run main loop
    player.run();
    
    // Cleanup
    bridge.stop();
    player.stop();
    
    return 0;
}