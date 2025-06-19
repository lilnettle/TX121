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
        perror("tcgetattr failed");
        return false;
    }
    
    // Точно як у pyserial - спочатку базові налаштування
    tty.c_cflag &= ~PARENB;   // No parity
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit  
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8 bits
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_cflag &= ~CRTSCTS;  // No hardware flow control
    
    // Raw mode - точно як у pyserial
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    
    // Timeout як у pyserial
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;  // 0.1 sec timeout
    
    // Спочатку встановити стандартну швидкість (pyserial робить так)
    cfsetispeed(&tty, B38400);  // Pyserial використовує B38400 як базу
    cfsetospeed(&tty, B38400);
    
    // Застосувати базові налаштування
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr failed");
        return false;
    }
    
    // Тепер кастомна швидкість - як у pyserial backend
    struct serial_struct ss;
    
    if (ioctl(fd, TIOCGSERIAL, &ss) != 0) {
        perror("TIOCGSERIAL failed");
        return false;
    }
    
    // Логіка pyserial для Linux:
    // 1. Зберігаємо оригінальні флаги
    unsigned int original_flags = ss.flags;
    
    // 2. Встановлюємо ASYNC_SPD_CUST флаг (як pyserial)
    ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
    
    // 3. Розраховуємо divisor (як у pyserial)
    ss.custom_divisor = ss.baud_base / 420000;
    
    // 4. Перевірка divisor (як у pyserial)
    if (ss.custom_divisor == 0) {
        ss.custom_divisor = 1;  // Мінімальне значення
    }
    
    std::cout << "🔧 PySerial-style config:\n";
    std::cout << "   baud_base: " << ss.baud_base << "\n";
    std::cout << "   custom_divisor: " << ss.custom_divisor << "\n";
    std::cout << "   calculated_baud: " << (ss.baud_base / ss.custom_divisor) << "\n";
    std::cout << "   flags: 0x" << std::hex << ss.flags << std::dec << "\n";
    
    // 5. Застосувати (як у pyserial)
    if (ioctl(fd, TIOCSSERIAL, &ss) != 0) {
        perror("TIOCSSERIAL failed");
        // Відновити оригінальні флаги
        ss.flags = original_flags;
        ioctl(fd, TIOCSSERIAL, &ss);
        return false;
    }
    
    // 6. Flush buffers (як у pyserial)
    tcflush(fd, TCIOFLUSH);
    
    std::cout << "✅ PySerial-style 420000 baud set successfully\n";
    return true;
}

// Альтернативний метод - прямий виклик як у Python
bool setup_baud_pyserial_way(int fd, int target_baud) {
    // Точна копія того, що робить pyserial.serialposix
    
    struct termios tty;
    tcgetattr(fd, &tty);
    
    // Встановити B38400 як базу для custom (стандарт pyserial)
    cfsetispeed(&tty, B38400);
    cfsetospeed(&tty, B38400);
    
    // 8N1 + raw mode
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 1;            // 0.1 seconds read timeout
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return false;
    }
    
    // Custom baud через ioctl (точно як pyserial)
    struct serial_struct serial;
    ioctl(fd, TIOCGSERIAL, &serial);
    
    serial.custom_divisor = serial.baud_base / target_baud;
    serial.flags &= ~ASYNC_SPD_MASK;
    serial.flags |= ASYNC_SPD_CUST;
    
    ioctl(fd, TIOCSSERIAL, &serial);
    
    return true;
}

bool try_connect() {
    std::cout << "🔌 Connecting (PySerial method) at " << CRSF_BAUD << " baud...\n";
    
    // Відкрити порти
    rx_fd = open(RX_PORT.c_str(), O_RDWR | O_NOCTTY);
    if (rx_fd < 0) {
        perror("Failed to open RX port");
        return false;
    }
    
    fc_fd = open(FC_PORT.c_str(), O_RDWR | O_NOCTTY);
    if (fc_fd < 0) {
        perror("Failed to open FC port");
        close(rx_fd);
        rx_fd = -1;
        return false;
    }
    
    // Метод 1: PySerial-style (основний)
    bool rx_ok = setup_baud_pyserial_way(rx_fd, CRSF_BAUD);
    bool fc_ok = setup_baud_pyserial_way(fc_fd, CRSF_BAUD);
    
    if (!rx_ok || !fc_ok) {
        std::cout << "❌ PySerial method failed, trying alternative...\n";
        
        // Метод 2: Детальний (резервний)
        rx_ok = setup_custom_baud_420000(rx_fd);
        fc_ok = setup_custom_baud_420000(fc_fd);
        
        if (!rx_ok || !fc_ok) {
            std::cout << "❌ All methods failed\n";
            close(rx_fd);
            close(fc_fd);
            rx_fd = fc_fd = -1;
            return false;
        }
    }
    
    std::cout << "✅ Connected at " << CRSF_BAUD << " baud (PySerial method)\n";
    std::cout << "📡 Bridge: " << RX_PORT << " → " << FC_PORT << "\n";
    return true;
}

// Додатково: функція для перевірки швидкості
void verify_baud_rate(int fd, const std::string& port_name) {
    struct serial_struct ss;
    if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
        int actual_baud = ss.baud_base / ss.custom_divisor;
        std::cout << "📊 " << port_name << " actual baud: " << actual_baud 
                  << " (flags: 0x" << std::hex << ss.flags << std::dec << ")\n";
    }
}

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