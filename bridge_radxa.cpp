#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>
#include <vector>
#include <cstring>
#include <csignal>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

// GStreamer
#include <gst/gst.h>
#include <glib.h>

// Фіксовані конфігурації
const std::string RTSP_URL = "rtsp://root:12345@192.168.0.100:554/stream1";
const int CRSF_BAUD = 420000;
const int VIDEO_WIDTH = 1280;
const int VIDEO_HEIGHT = 720;
const int VIDEO_FPS = 30;
const std::string RX_PORT = "/dev/ttyUSB1";  // RX приймач
const std::string FC_PORT = "/dev/ttyUSB0";  // FC контролер

// Структура статистики bridge
struct BridgeStats {
    std::atomic<bool> bridge_active{false};
    std::atomic<int> rx_packets{0};
    std::atomic<int> fc_packets{0};
};

// Простий CRSF Bridge клас з підтримкою нестандартних швидкостей
class SimpleCRSFBridge {
private:
    std::string rx_port;
    std::string fc_port;
    int baud_rate;
    
    int rx_fd = -1;
    int fc_fd = -1;
    std::atomic<bool> running{false};
    
    struct {
        std::atomic<int> rx_packets{0};
        std::atomic<int> fc_packets{0};
        std::atomic<int> errors{0};
    } stats;
    
    std::unique_ptr<std::thread> bridge_thread;

public:
    SimpleCRSFBridge(const std::string& rx_port, const std::string& fc_port, int baud_rate)
        : rx_port(rx_port), fc_port(fc_port), baud_rate(baud_rate) {}
    
    ~SimpleCRSFBridge() {
        stop();
    }
    
    // Налаштування серійного порту з підтримкою нестандартних швидкостей
    bool setup_serial_port(int fd, int baud) {
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(fd, &tty) != 0) {
            std::cerr << "Error getting terminal attributes\n";
            return false;
        }
        
        // Спробувати стандартні швидкості спочатку
        speed_t speed = B0;
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
                // Для нестандартних швидкостей (наприклад, 420000)
                std::cout << "⚡ Setting custom baud rate: " << baud << "\n";
                speed = B38400; // Використаємо як базу для custom speed
                break;
        }
        
        // Налаштування базових параметрів
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        
        // 8N1 конфігурація
        tty.c_cflag &= ~PARENB;   // No parity
        tty.c_cflag &= ~CSTOPB;   // 1 stop bit
        tty.c_cflag &= ~CSIZE;    // Clear size bits
        tty.c_cflag |= CS8;       // 8 bits
        tty.c_cflag &= ~CRTSCTS;  // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Enable reading
        
        // Raw mode
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        
        // Timeouts
        tty.c_cc[VMIN] = 0;   // Non-blocking read
        tty.c_cc[VTIME] = 1;  // 0.1 seconds timeout
        
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting terminal attributes\n";
            return false;
        }
        
        // Для нестандартних швидкостей використовуємо ioctl
        if (speed == B38400 && baud != 38400) {
            struct serial_struct ss;
            if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
                ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
                ss.custom_divisor = ss.baud_base / baud;
                if (ioctl(fd, TIOCSSERIAL, &ss) == 0) {
                    std::cout << "✅ Custom baud rate " << baud << " set successfully\n";
                } else {
                    std::cout << "⚠️ Custom baud rate failed, using 115200 fallback\n";
                    // Fallback to 115200
                    cfsetispeed(&tty, B115200);
                    cfsetospeed(&tty, B115200);
                    tcsetattr(fd, TCSANOW, &tty);
                }
            }
        }
        
        // Flush buffers
        tcflush(fd, TCIOFLUSH);
        
        return true;
    }
    
    // Підключення
    bool connect() {
        std::cout << "🔌 Connecting CRSF bridge at " << baud_rate << " baud...\n";
        
        // Відкрити RX порт
        rx_fd = open(rx_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (rx_fd < 0) {
            std::cerr << "❌ Failed to open RX port: " << rx_port << "\n";
            return false;
        }
        
        // Відкрити FC порт
        fc_fd = open(fc_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fc_fd < 0) {
            std::cerr << "❌ Failed to open FC port: " << fc_port << "\n";
            close(rx_fd);
            rx_fd = -1;
            return false;
        }
        
        // Налаштувати обидва порти
        if (!setup_serial_port(rx_fd, baud_rate) || !setup_serial_port(fc_fd, baud_rate)) {
            disconnect();
            return false;
        }
        
        std::cout << "✅ Bridge connected at " << baud_rate << " baud\n";
        std::cout << "🌉 Bridge: " << rx_port << " → " << fc_port << "\n";
        return true;
    }
    
    // Відключення
    void disconnect() {
        if (rx_fd >= 0) {
            close(rx_fd);
            rx_fd = -1;
        }
        if (fc_fd >= 0) {
            close(fc_fd);
            fc_fd = -1;
        }
    }
    
    // Головний цикл bridge
    void bridge_loop(BridgeStats& bridge_stats) {
        std::cout << "🔄 Bridge thread started\n";
        
        char buffer[1024];
        
        while (running) {
            try {
                // USB1 → USB0 (RX → FC) - передача команд управління
                if (rx_fd >= 0) {
                    ssize_t bytes = read(rx_fd, buffer, sizeof(buffer));
                    if (bytes > 0) {
                        if (fc_fd >= 0) {
                            write(fc_fd, buffer, bytes);
                        }
                        stats.rx_packets += bytes;
                    }
                }
                
                // USB0 → USB1 (FC → RX) - телеметрія назад
                if (fc_fd >= 0) {
                    ssize_t bytes = read(fc_fd, buffer, sizeof(buffer));
                    if (bytes > 0) {
                        if (rx_fd >= 0) {
                            write(rx_fd, buffer, bytes);
                        }
                        stats.fc_packets += bytes;
                    }
                }
                
                // Оновити статистику
                bridge_stats.bridge_active = true;
                bridge_stats.rx_packets = stats.rx_packets;
                bridge_stats.fc_packets = stats.fc_packets;
                
                std::this_thread::sleep_for(std::chrono::microseconds(500)); // 0.5ms
                
            } catch (const std::exception& e) {
                std::cout << "❌ Bridge error: " << e.what() << "\n";
                stats.errors++;
                bridge_stats.bridge_active = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    
    // Запустити bridge
    bool start(BridgeStats& bridge_stats) {
        if (!connect()) {
            return false;
        }
        
        running = true;
        
        // Запустити bridge потік
        bridge_thread = std::make_unique<std::thread>(&SimpleCRSFBridge::bridge_loop, this, std::ref(bridge_stats));
        
        std::cout << "🚀 CRSF Bridge running!\n";
        return true;
    }
    
    // Зупинити bridge
    void stop() {
        running = false;
        if (bridge_thread && bridge_thread->joinable()) {
            bridge_thread->join();
        }
        disconnect();
        std::cout << "⏹️ Bridge stopped\n";
    }
};

// Головний клас чистого відео з bridge
class CleanVideoWithBridge {
private:
    bool enable_bridge;
    
    GstElement* pipeline = nullptr;
    GMainLoop* loop = nullptr;
    std::atomic<bool> running{false};
    
    BridgeStats bridge_stats;
    
    std::unique_ptr<SimpleCRSFBridge> bridge;
    std::unique_ptr<std::thread> status_thread;

public:
    CleanVideoWithBridge(bool enable_bridge = true)
        : enable_bridge(enable_bridge) {
        
        // Створити bridge якщо потрібен
        if (enable_bridge) {
            bridge = std::make_unique<SimpleCRSFBridge>(RX_PORT, FC_PORT, CRSF_BAUD);
        }
    }
    
    ~CleanVideoWithBridge() {
        if (running) {
            stop();
        }
    }
    
    // Створити оптимізований GStreamer pipeline
    std::string create_gstreamer_pipeline() {
        // Оптимізований pipeline для фіксованих параметрів
        std::string pipeline_str = 
            "rtspsrc location=" + RTSP_URL + " "
            "latency=0 "
            "drop-on-latency=true "
            "do-retransmission=false "
            "timeout=5000000 "
            "protocols=tcp+udp-mcast+udp ! "
            "queue max-size-buffers=3 leaky=downstream ! "
            "rtph264depay ! "
            "queue max-size-buffers=2 leaky=downstream ! ";
        
        // Спробувати MPP decoder для Rockchip, fallback на software
        pipeline_str += 
            "mpph264dec ! "
            "queue max-size-buffers=2 leaky=downstream ! "
            "videoscale method=nearest-neighbour ! "
            "video/x-raw,width=" + std::to_string(VIDEO_WIDTH) + ",height=" + std::to_string(VIDEO_HEIGHT) + " ! "
            "videoconvert ! "
            "queue max-size-buffers=2 leaky=downstream ! ";
        
        // KMS вихід для мінімальної затримки
        pipeline_str += 
            "kmssink sync=false max-lateness=0 qos=false processing-deadline=0 render-delay=0 async=false";
        
        return pipeline_str;
    }
    
    // Обробник повідомлень GStreamer
    static gboolean bus_callback(GstBus* bus, GstMessage* message, gpointer data) {
        CleanVideoWithBridge* self = static_cast<CleanVideoWithBridge*>(data);
        
        switch (GST_MESSAGE_TYPE(message)) {
            case GST_MESSAGE_ERROR: {
                GError* err;
                gchar* debug;
                gst_message_parse_error(message, &err, &debug);
                std::cout << "❌ GStreamer Error: " << err->message << "\n";
                
                // Спробувати software decoder як fallback
                std::string error_str = err->message;
                if (error_str.find("mpph264dec") != std::string::npos) {
                    std::cout << "🔄 MPP decoder failed, trying software decoder...\n";
                    // TODO: Перезапустити з software decoder
                } else {
                    g_main_loop_quit(self->loop);
                }
                
                g_error_free(err);
                g_free(debug);
                break;
            }
            case GST_MESSAGE_EOS:
                std::cout << "📺 End of stream\n";
                g_main_loop_quit(self->loop);
                break;
            case GST_MESSAGE_WARNING: {
                GError* warn;
                gchar* debug;
                gst_message_parse_warning(message, &warn, &debug);
                std::cout << "⚠️ Warning: " << warn->message << "\n";
                g_error_free(warn);
                g_free(debug);
                break;
            }
            case GST_MESSAGE_STATE_CHANGED:
                if (GST_MESSAGE_SRC(message) == GST_OBJECT(self->pipeline)) {
                    GstState old_state, new_state, pending;
                    gst_message_parse_state_changed(message, &old_state, &new_state, &pending);
                    std::cout << "🎬 Pipeline state: " << gst_element_state_get_name(old_state) 
                              << " → " << gst_element_state_get_name(new_state) << "\n";
                }
                break;
            case GST_MESSAGE_BUFFERING: {
                gint percent;
                gst_message_parse_buffering(message, &percent);
                if (percent < 100) {
                    gst_element_set_state(self->pipeline, GST_STATE_PAUSED);
                } else {
                    gst_element_set_state(self->pipeline, GST_STATE_PLAYING);
                }
                break;
            }
            default:
                break;
        }
        
        return TRUE;
    }
    
    // Налаштувати pipeline
    bool setup_pipeline() {
        std::string pipeline_str = create_gstreamer_pipeline();
        
        std::cout << "🎬 Creating video pipeline for " << VIDEO_WIDTH << "x" << VIDEO_HEIGHT << "@" << VIDEO_FPS << "fps\n";
        std::cout << "📡 RTSP: " << RTSP_URL << "\n";
        
        GError* error = nullptr;
        pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
        if (error) {
            std::cout << "❌ Failed to create pipeline: " << error->message << "\n";
            std::cout << "🔄 Trying software decoder fallback...\n";
            
            // Fallback pipeline з software decoder
            std::string fallback_pipeline = 
                "rtspsrc location=" + RTSP_URL + " "
                "latency=0 drop-on-latency=true do-retransmission=false timeout=5000000 "
                "protocols=tcp+udp-mcast+udp ! "
                "queue max-size-buffers=3 leaky=downstream ! "
                "rtph264depay ! "
                "queue max-size-buffers=2 leaky=downstream ! "
                "avdec_h264 max-threads=2 ! "
                "videoscale method=nearest-neighbour ! "
                "video/x-raw,width=" + std::to_string(VIDEO_WIDTH) + ",height=" + std::to_string(VIDEO_HEIGHT) + " ! "
                "videoconvert ! "
                "queue max-size-buffers=2 leaky=downstream ! "
                "ximagesink sync=false force-aspect-ratio=true qos=false async=false";
            
            g_error_free(error);
            error = nullptr;
            pipeline = gst_parse_launch(fallback_pipeline.c_str(), &error);
            
            if (error) {
                std::cout << "❌ Software decoder also failed: " << error->message << "\n";
                g_error_free(error);
                return false;
            }
            std::cout << "✅ Using software decoder\n";
        } else {
            std::cout << "✅ Using hardware MPP decoder\n";
        }
        
        // Message handler
        GstBus* bus = gst_element_get_bus(pipeline);
        gst_bus_add_watch(bus, bus_callback, this);
        gst_object_unref(bus);
        
        return true;
    }
    
    // Статус системи
    void print_status() {
        while (running) {
            // Bridge статус
            std::string bridge_status, bridge_info;
            if (enable_bridge) {
                bridge_status = bridge_stats.bridge_active ? "🌉 ACTIVE" : "🌉 INACTIVE";
                bridge_info = "RX→FC: " + std::to_string(bridge_stats.rx_packets.load()) + 
                             " | FC→RX: " + std::to_string(bridge_stats.fc_packets.load());
            } else {
                bridge_status = "🌉 DISABLED";
                bridge_info = "";
            }
            
            std::cout << "\r📺 " << VIDEO_WIDTH << "x" << VIDEO_HEIGHT << " | " 
                      << bridge_status << " | " << bridge_info << "    " << std::flush;
            
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
    
    // Перевірити існування файлу
    bool file_exists(const std::string& path) {
        struct stat buffer;
        return (stat(path.c_str(), &buffer) == 0);
    }
    
    // Запустити систему
    bool run() {
        // Перевірити порти якщо bridge увімкнений
        if (enable_bridge) {
            if (!file_exists(RX_PORT)) {
                std::cout << "❌ RX port " << RX_PORT << " not found!\n";
                return false;
            }
            if (!file_exists(FC_PORT)) {
                std::cout << "❌ FC port " << FC_PORT << " not found!\n";
                return false;
            }
        }
        
        // Налаштувати pipeline
        if (!setup_pipeline()) {
            return false;
        }
        
        running = true;
        
        // Запустити bridge (якщо потрібен)
        if (enable_bridge && bridge) {
            if (!bridge->start(bridge_stats)) {
                std::cout << "❌ Failed to start bridge\n";
                return false;
            }
        }
        
        // Запустити статус потік
        status_thread = std::make_unique<std::thread>(&CleanVideoWithBridge::print_status, this);
        
        // Запустити pipeline
        GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            std::cout << "❌ Failed to start pipeline\n";
            return false;
        }
        
        // Головний loop
        loop = g_main_loop_new(nullptr, FALSE);
        
        std::cout << "\n🎬 Clean Video + CRSF Bridge System\n";
        std::cout << "================================\n";
        std::cout << "📺 Video: " << RTSP_URL << "\n";
        std::cout << "📐 Resolution: " << VIDEO_WIDTH << "x" << VIDEO_HEIGHT << "@" << VIDEO_FPS << "fps\n";
        
        if (enable_bridge) {
            std::cout << "🌉 Bridge: " << RX_PORT << " → " << FC_PORT << " @ " << CRSF_BAUD << " baud\n";
            std::cout << "   ⚡ Control: RX → FC (ultra-low latency)\n";
            std::cout << "   📡 Telemetry: FC → RX\n";
        } else {
            std::cout << "🌉 Bridge: DISABLED\n";
        }
        
        std::cout << "📺 Video: Clean passthrough (no OSD)\n";
        std::cout << "\nPress Ctrl+C to stop\n\n";
        
        g_main_loop_run(loop);
        
        return true;
    }
    
    // Зупинити систему
    void stop() {
        running = false;
        
        if (bridge && enable_bridge) {
            bridge->stop();
        }
        
        if (status_thread && status_thread->joinable()) {
            status_thread->join();
        }
        
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
            pipeline = nullptr;
        }
        
        if (loop) {
            g_main_loop_quit(loop);
            g_main_loop_unref(loop);
            loop = nullptr;
        }
    }
};

// Глобальний вказівник для обробки сигналів
static CleanVideoWithBridge* global_system = nullptr;

// Обробник сигналів
void signal_handler(int signal) {
    std::cout << "\n🛑 Stopping system...\n";
    if (global_system) {
        global_system->stop();
    }
    exit(0);
}

// Головна функція
int main(int argc, char* argv[]) {
    // Ініціалізація GStreamer
    gst_init(&argc, &argv);
    
    // Встановити обробник сигналів
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Простий парсинг аргументів
    bool enable_bridge = true;
    bool show_help = false;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--no-bridge") {
            enable_bridge = false;
        } else if (arg == "-h" || arg == "--help") {
            show_help = true;
        }
    }
    
    if (show_help) {
        std::cout << "🎬 Clean Video + CRSF Bridge Control (Simplified)\n";
        std::cout << "================================================\n";
        std::cout << "Fixed Configuration:\n";
        std::cout << "  📡 RTSP: " << RTSP_URL << "\n";
        std::cout << "  📐 Video: " << VIDEO_WIDTH << "x" << VIDEO_HEIGHT << "@" << VIDEO_FPS << "fps\n";
        std::cout << "  ⚡ CRSF: " << CRSF_BAUD << " baud\n";
        std::cout << "  🔌 Ports: " << RX_PORT << " → " << FC_PORT << "\n";
        std::cout << "\nUsage: " << argv[0] << " [options]\n";
        std::cout << "Options:\n";
        std::cout << "  --no-bridge    Disable CRSF bridge (video only)\n";
        std::cout << "  -h, --help     Show this help\n";
        return 0;
    }
    
    std::cout << "🎬 CLEAN VIDEO + CRSF BRIDGE (SIMPLIFIED)\n";
    std::cout << "=========================================\n";
    std::cout << "📡 RTSP Stream: " << RTSP_URL << "\n";
    std::cout << "📐 Video Output: " << VIDEO_WIDTH << "x" << VIDEO_HEIGHT << "@" << VIDEO_FPS << "fps\n";
    std::cout << "⚡ CRSF Baud: " << CRSF_BAUD << " (with custom speed support)\n";
    if (enable_bridge) {
        std::cout << "🌉 Bridge: " << RX_PORT << " → " << FC_PORT << "\n";
    } else {
        std::cout << "🌉 Bridge: DISABLED\n";
    }
    std::cout << "🎯 Mode: Ultra-low latency (hardware accelerated)\n\n";
    
    // Створити систему
    CleanVideoWithBridge system(enable_bridge);
    
    // Встановити глобальний вказівник
    global_system = &system;
    
    // Запустити систему
    if (!system.run()) {
        std::cout << "❌ Failed to start system\n";
        return 1;
    }
    
    return 0;
}