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

// Boost.Asio
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>

// GStreamer
#include <gst/gst.h>
#include <glib.h>

using namespace boost::asio;

// Конфігурація
const std::string RTSP_URL = "rtsp://root:12345@192.168.0.100:554/stream1";
const std::string RX_PORT = "/dev/ttyUSB1";   // RX приймач
const std::string FC_PORT = "/dev/ttyUSB0";   // FC
const int CRSF_BAUD = 420000;  // Кастомна швидкість CRSF

// Структура для детальної конфігурації порту
struct SerialConfig {
    int baud_rate = 420000;
    int data_bits = 8;
    int stop_bits = 1;
    char parity = 'N';  // 'N', 'E', 'O'
    bool flow_control = false;
    bool xonxoff = false;
    bool rtscts = false;
    bool dsrdtr = false;
    
    // Timeout налаштування (як у PySerial)
    double timeout = 0.1;      // read timeout в секундах
    double write_timeout = 0.1; // write timeout в секундах
    
    // Буферні налаштування
    int read_buffer_size = 4096;
    int write_buffer_size = 4096;
    
    // Додаткові налаштування
    bool exclusive = true;      // ексклюзивний доступ
    bool low_latency = true;    // мінімальна затримка
};

// Клас для роботи з серійними портами в стилі PySerial
class PySerialLikePort {
private:
    int fd = -1;
    std::string port_name;
    SerialConfig config;
    bool is_connected = false;
    
    // Налаштування termios структури
    bool configure_termios() {
        struct termios tty;
        
        if (tcgetattr(fd, &tty) < 0) {
            std::cerr << "❌ Помилка tcgetattr для " << port_name << std::endl;
            return false;
        }
        
        // Очищення всіх попередніх налаштувань
        memset(&tty, 0, sizeof(tty));
        
        // === НАЛАШТУВАННЯ ВВОДУ (cflag) ===
        tty.c_cflag |= CREAD | CLOCAL;  // Увімкнути читання та ігнорувати modem control lines
        
        // Розмір символу
        tty.c_cflag &= ~CSIZE;
        switch (config.data_bits) {
            case 5: tty.c_cflag |= CS5; break;
            case 6: tty.c_cflag |= CS6; break;
            case 7: tty.c_cflag |= CS7; break;
            case 8: tty.c_cflag |= CS8; break;
            default: tty.c_cflag |= CS8; break;
        }
        
        // Парність
        switch (config.parity) {
            case 'E': case 'e':
                tty.c_cflag |= PARENB;
                tty.c_cflag &= ~PARODD;
                break;
            case 'O': case 'o':
                tty.c_cflag |= PARENB;
                tty.c_cflag |= PARODD;
                break;
            case 'N': case 'n':
            default:
                tty.c_cflag &= ~PARENB;
                break;
        }
        
        // Stop bits
        if (config.stop_bits == 2) {
            tty.c_cflag |= CSTOPB;
        } else {
            tty.c_cflag &= ~CSTOPB;
        }
        
        // Flow control
        if (config.rtscts) {
            tty.c_cflag |= CRTSCTS;
        } else {
            tty.c_cflag &= ~CRTSCTS;
        }
        
        // === НАЛАШТУВАННЯ ВВОДУ (iflag) ===
        tty.c_iflag = 0;  // Сирий ввід
        
        if (config.xonxoff) {
            tty.c_iflag |= IXON | IXOFF;
        } else {
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        }
        
        // Вимкнути обробку символів
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        
        // === НАЛАШТУВАННЯ ВИВОДУ (oflag) ===
        tty.c_oflag = 0;  // Сирий вивід
        
        // === НАЛАШТУВАННЯ LOCAL (lflag) ===
        tty.c_lflag = 0;  // Вимкнути канонічний режим та echo
        
        // === НАЛАШТУВАННЯ TIMEOUT ===
        // VMIN та VTIME для неблокуючого читання
        tty.c_cc[VMIN] = 0;   // Мінімум символів для читання
        tty.c_cc[VTIME] = (cc_t)(config.timeout * 10);  // Timeout в десятих секунди
        
        // Застосування налаштувань
        if (tcsetattr(fd, TCSANOW, &tty) < 0) {
            std::cerr << "❌ Помилка tcsetattr для " << port_name << std::endl;
            return false;
        }
        
        return true;
    }
    
    // Налаштування кастомної швидкості
    bool set_custom_baud_rate(int baud) {
        struct serial_struct ss;
        
        if (ioctl(fd, TIOCGSERIAL, &ss) < 0) {
            std::cerr << "❌ Помилка отримання serial_struct для " << port_name << std::endl;
            return false;
        }
        
        // Налаштування кастомної швидкості
        ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
        ss.custom_divisor = ss.baud_base / baud;
        
        if (ioctl(fd, TIOCSSERIAL, &ss) < 0) {
            std::cerr << "❌ Помилка встановлення кастомної швидкості " << baud 
                      << " для " << port_name << std::endl;
            return false;
        }
        
        // Верифікація
        if (ioctl(fd, TIOCGSERIAL, &ss) < 0) {
            return false;
        }
        
        int actual_baud = ss.baud_base / ss.custom_divisor;
        std::cout << "✅ " << port_name << ": встановлено " << actual_baud 
                  << " baud (запитано " << baud << ")" << std::endl;
        
        return true;
    }
    
    // Налаштування розмірів буферів
    bool configure_buffers() {
        // Налаштування розмірів буферів ядра
        if (ioctl(fd, TIOCOUTQ) >= 0) {  // Перевірка підтримки
            // Спроба налаштувати буфери (не всі драйвери підтримують)
            struct serial_struct ss;
            if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
                // Деякі драйвери дозволяють налаштовувати розміри буферів
                std::cout << "📊 " << port_name << ": baud_base=" << ss.baud_base 
                          << ", custom_divisor=" << ss.custom_divisor << std::endl;
            }
        }
        
        return true;
    }
    
    // Налаштування низької затримки
    bool configure_low_latency() {
        if (!config.low_latency) return true;
        
        struct serial_struct ss;
        if (ioctl(fd, TIOCGSERIAL, &ss) < 0) {
            return false;
        }
        
        ss.flags |= ASYNC_LOW_LATENCY;
        
        if (ioctl(fd, TIOCSSERIAL, &ss) < 0) {
            std::cout << "⚠️ " << port_name << ": не вдалося встановити LOW_LATENCY" << std::endl;
            return false;
        }
        
        std::cout << "⚡ " << port_name << ": увімкнено LOW_LATENCY режим" << std::endl;
        return true;
    }

public:
    PySerialLikePort(const std::string& port, const SerialConfig& cfg = SerialConfig()) 
        : port_name(port), config(cfg) {}
    
    ~PySerialLikePort() {
        close();
    }
    
    // Підключення до порту
    bool open() {
        if (is_connected) {
            std::cout << "⚠️ " << port_name << " вже підключений" << std::endl;
            return true;
        }
        
        std::cout << "🔌 Підключення до " << port_name << "..." << std::endl;
        
        // Відкриття порту
        int flags = O_RDWR | O_NOCTTY;
        if (!config.exclusive) {
            flags |= O_NONBLOCK;
        }
        
        fd = ::open(port_name.c_str(), flags);
        if (fd < 0) {
            std::cerr << "❌ Не вдалося відкрити " << port_name 
                      << ": " << strerror(errno) << std::endl;
            return false;
        }
        
        // Ексклюзивний доступ
        if (config.exclusive) {
            if (ioctl(fd, TIOCEXCL) < 0) {
                std::cout << "⚠️ " << port_name << ": не вдалося встановити ексклюзивний доступ" << std::endl;
            }
        }
        
        // Налаштування порту
        if (!configure_termios()) {
            ::close(fd);
            fd = -1;
            return false;
        }
        
        // Налаштування кастомної швидкості
        if (!set_custom_baud_rate(config.baud_rate)) {
            ::close(fd);
            fd = -1;
            return false;
        }
        
        // Додаткові налаштування
        configure_buffers();
        configure_low_latency();
        
        // Очищення буферів
        tcflush(fd, TCIOFLUSH);
        
        is_connected = true;
        std::cout << "✅ " << port_name << " підключено успішно" << std::endl;
        
        return true;
    }
    
    // Закриття порту
    void close() {
        if (is_connected && fd >= 0) {
            tcflush(fd, TCIOFLUSH);
            ::close(fd);
            fd = -1;
            is_connected = false;
            std::cout << "🔌 " << port_name << " закрито" << std::endl;
        }
    }
    
    // Отримання file descriptor для Boost.Asio
    int get_fd() const { return fd; }
    bool connected() const { return is_connected; }
    const std::string& get_port_name() const { return port_name; }
    
    // Прямий запис/читання (для тестування)
    ssize_t write_direct(const void* data, size_t size) {
        if (!is_connected) return -1;
        return ::write(fd, data, size);
    }
    
    ssize_t read_direct(void* data, size_t size) {
        if (!is_connected) return -1;
        return ::read(fd, data, size);
    }
    
    // Очищення буферів
    void flush() {
        if (is_connected) {
            tcflush(fd, TCIOFLUSH);
        }
    }
    
    // Отримання статистики порту
    void print_port_info() {
        if (!is_connected) return;
        
        struct serial_struct ss;
        if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
            std::cout << "📊 " << port_name << " інформація:" << std::endl;
            std::cout << "   Baud base: " << ss.baud_base << std::endl;
            std::cout << "   Custom divisor: " << ss.custom_divisor << std::endl;
            std::cout << "   Actual baud: " << (ss.custom_divisor > 0 ? ss.baud_base / ss.custom_divisor : 0) << std::endl;
            std::cout << "   Flags: 0x" << std::hex << ss.flags << std::dec << std::endl;
        }
        
        // Статистика буферів
        int input_queue = 0, output_queue = 0;
        if (ioctl(fd, TIOCINQ, &input_queue) == 0) {
            std::cout << "   Input queue: " << input_queue << " bytes" << std::endl;
        }
        if (ioctl(fd, TIOCOUTQ, &output_queue) == 0) {
            std::cout << "   Output queue: " << output_queue << " bytes" << std::endl;
        }
    }
};

// Асинхронний CRSF Bridge з покращеними портами
class AsyncCrsfBridge {
private:
    io_context io_ctx;
    std::unique_ptr<serial_port> rx_port, fc_port;
    std::unique_ptr<PySerialLikePort> rx_native, fc_native;
    deadline_timer stats_timer;
    std::thread io_thread;
    
    // Буфери для асинхронного читання
    static constexpr size_t BUFFER_SIZE = 4096;  // Збільшений буфер
    std::vector<uint8_t> rx_buffer, fc_buffer;
    
    // Статистика
    std::atomic<long> rx_to_fc_bytes{0}, fc_to_rx_bytes{0}, errors{0};
    std::atomic<bool> running{false};
    
    // Callbacks для обробки помилок
    std::function<void(const std::string&)> error_callback;

public:
    AsyncCrsfBridge() 
        : stats_timer(io_ctx), rx_buffer(BUFFER_SIZE), fc_buffer(BUFFER_SIZE) {}
    
    ~AsyncCrsfBridge() {
        stop();
    }
    
    // Встановлення callback для помилок
    void set_error_callback(std::function<void(const std::string&)> callback) {
        error_callback = callback;
    }
    
    // Відкриття та налаштування портів
    bool connect() {
        try {
            std::cout << "🔌 Підключення портів з PySerial-подібними налаштуваннями..." << std::endl;
            
            // Конфігурація порту
            SerialConfig config;
            config.baud_rate = CRSF_BAUD;
            config.timeout = 0.01;      // 10ms timeout
            config.write_timeout = 0.01;
            config.low_latency = true;
            config.exclusive = true;
            
            // Створення нативних портів
            rx_native = std::make_unique<PySerialLikePort>(RX_PORT, config);
            fc_native = std::make_unique<PySerialLikePort>(FC_PORT, config);
            
            // Підключення нативних портів
            if (!rx_native->open() || !fc_native->open()) {
                return false;
            }
            
            // Створення Boost.Asio serial_port з нативних дескрипторів
            rx_port = std::make_unique<serial_port>(io_ctx);
            fc_port = std::make_unique<serial_port>(io_ctx);
            
            // Призначення нативних дескрипторів
            rx_port->assign(rx_native->get_fd());
            fc_port->assign(fc_native->get_fd());
            
            std::cout << "✅ Порти підключено успішно!" << std::endl;
            std::cout << "📡 Bridge: " << RX_PORT << " ↔ " << FC_PORT << std::endl;
            
            // Вивід детальної інформації про порти
            rx_native->print_port_info();
            fc_native->print_port_info();
            
            return true;
            
        } catch (boost::system::system_error& e) {
            std::cout << "❌ Помилка підключення: " << e.what() << std::endl;
            return false;
        }
    }
    
    // Запуск асинхронного bridge
    bool start() {
        if (!connect()) {
            return false;
        }
        
        running = true;
        
        // Запуск асинхронного читання з обох портів
        start_rx_read();
        start_fc_read();
        
        // Запуск таймера статистики
        start_stats_timer();
        
        // Запуск IO потоку
        io_thread = std::thread([this]() {
            try {
                io_ctx.run();
            } catch (std::exception& e) {
                std::cout << "❌ IO context error: " << e.what() << std::endl;
                if (error_callback) {
                    error_callback("IO context error: " + std::string(e.what()));
                }
            }
        });
        
        std::cout << "🚀 Асинхронний bridge запущено!" << std::endl;
        return true;
    }
    
    // Зупинка bridge
    void stop() {
        if (!running) return;
        
        running = false;
        
        // Зупинка IO контексту
        io_ctx.stop();
        
        // Закриття портів
        boost::system::error_code ec;
        if (rx_port && rx_port->is_open()) {
            rx_port->cancel(ec);
            rx_port->close(ec);
        }
        if (fc_port && fc_port->is_open()) {
            fc_port->cancel(ec);
            fc_port->close(ec);
        }
        
        // Закриття нативних портів
        if (rx_native) rx_native->close();
        if (fc_native) fc_native->close();
        
        // Чекання завершення IO потоку
        if (io_thread.joinable()) {
            io_thread.join();
        }
        
        std::cout << "⏹️ Асинхронний bridge зупинено" << std::endl;
        print_final_stats();
    }
    
    // Отримання поточної швидкості
    int get_baud() const { return CRSF_BAUD; }
    
    // Отримання статистики
    void get_stats(long& rx_to_fc, long& fc_to_rx, long& error_count) const {
        rx_to_fc = rx_to_fc_bytes.load();
        fc_to_rx = fc_to_rx_bytes.load();
        error_count = errors.load();
    }

private:
    // Асинхронне читання з RX порту (USB1 → USB0)
    void start_rx_read() {
        if (!running || !rx_port) return;
        
        rx_port->async_read_some(
            buffer(rx_buffer),
            [this](const boost::system::error_code& error, size_t bytes_transferred) {
                handle_rx_read(error, bytes_transferred);
            });
    }
    
    void handle_rx_read(const boost::system::error_code& error, size_t bytes_transferred) {
        if (!running) return;
        
        if (!error && bytes_transferred > 0) {
            // Передача даних від RX до FC (USB1 → USB0)
            async_write(*fc_port,
                buffer(rx_buffer, bytes_transferred),
                [this, bytes_transferred](const boost::system::error_code& error, size_t bytes_written) {
                    handle_rx_write(error, bytes_written, bytes_transferred);
                });
            
        } else if (error) {
            handle_error("RX read", error);
        }
        
        // Продовження читання
        if (running && rx_port && rx_port->is_open()) {
            start_rx_read();
        }
    }
    
    void handle_rx_write(const boost::system::error_code& error, 
                        size_t bytes_written, size_t expected_bytes) {
        if (!error) {
            rx_to_fc_bytes += bytes_written;
            if (bytes_written != expected_bytes) {
                std::cout << "⚠️ RX→FC: записано " << bytes_written 
                          << " з " << expected_bytes << " байт" << std::endl;
            }
        } else {
            handle_error("RX write", error);
        }
    }
    
    // Асинхронне читання з FC порту (USB0 → USB1)
    void start_fc_read() {
        if (!running || !fc_port) return;
        
        fc_port->async_read_some(
            buffer(fc_buffer),
            [this](const boost::system::error_code& error, size_t bytes_transferred) {
                handle_fc_read(error, bytes_transferred);
            });
    }
    
    void handle_fc_read(const boost::system::error_code& error, size_t bytes_transferred) {
        if (!running) return;
        
        if (!error && bytes_transferred > 0) {
            // Передача телеметрії від FC до RX (USB0 → USB1)
            async_write(*rx_port,
                buffer(fc_buffer, bytes_transferred),
                [this, bytes_transferred](const boost::system::error_code& error, size_t bytes_written) {
                    handle_fc_write(error, bytes_written, bytes_transferred);
                });
            
        } else if (error) {
            handle_error("FC read", error);
        }
        
        // Продовження читання
        if (running && fc_port && fc_port->is_open()) {
            start_fc_read();
        }
    }
    
    void handle_fc_write(const boost::system::error_code& error, 
                        size_t bytes_written, size_t expected_bytes) {
        if (!error) {
            fc_to_rx_bytes += bytes_written;
            if (bytes_written != expected_bytes) {
                std::cout << "⚠️ FC→RX: записано " << bytes_written 
                          << " з " << expected_bytes << " байт" << std::endl;
            }
        } else {
            handle_error("FC write", error);
        }
    }
    
    // Обробка помилок
    void handle_error(const std::string& operation, const boost::system::error_code& error) {
        errors++;
        std::string error_msg = operation + " error: " + error.message();
        std::cout << "❌ " << error_msg << std::endl;
        
        if (error_callback) {
            error_callback(error_msg);
        }
        
        // При критичних помилках зупиняємо bridge
        if (error == boost::asio::error::operation_aborted ||
            error == boost::asio::error::bad_descriptor) {
            std::cout << "🛑 Критична помилка, зупинка bridge..." << std::endl;
            stop();
        }
    }
    
    // Таймер статистики
    void start_stats_timer() {
        stats_timer.expires_from_now(boost::posix_time::seconds(5));
        stats_timer.async_wait(
            [this](const boost::system::error_code& error) {
                handle_stats_timer(error);
            });
    }
    
    void handle_stats_timer(const boost::system::error_code& error) {
        if (!error && running) {
            print_stats();
            start_stats_timer(); // Наступний виклик через 5 секунд
        }
    }
    
    void print_stats() {
        std::cout << "📊 RX→FC: " << rx_to_fc_bytes.load() << " байт | "
                  << "FC→RX: " << fc_to_rx_bytes.load() << " байт | "
                  << "Помилки: " << errors.load() << std::endl;
    }
    
    void print_final_stats() {
        std::cout << "\n📋 Фінальна статистика:" << std::endl;
        std::cout << "   RX→FC: " << rx_to_fc_bytes.load() << " байт" << std::endl;
        std::cout << "   FC→RX: " << fc_to_rx_bytes.load() << " байт" << std::endl;
        std::cout << "   Помилки: " << errors.load() << std::endl;
    }
};

// Простий відеоплеєр (залишається незмінним)
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
                        std::cout << "🎬 Відео відтворюється!" << std::endl;
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
        std::cout << "🎬 Запуск відео..." << std::endl;
        
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
static AsyncCrsfBridge* global_bridge = nullptr;
static SimpleVideoPlayer* global_player = nullptr;

void signal_handler(int /*signal*/) {
    std::cout << "\n🛑 Зупинка..." << std::endl;
    if (global_bridge) global_bridge->stop();
    if (global_player) global_player->stop();
    exit(0);
}

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "🌉 ASYNC CRSF BRIDGE (PySerial-подібний)" << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << "Конфігурація:" << std::endl;
    std::cout << "  RX Input:  " << RX_PORT << std::endl;
    std::cout << "  FC Output: " << FC_PORT << std::endl;
    std::cout << "  Напрямок: USB1 ↔ USB0 (двосторонній)" << std::endl;
    std::cout << "  Швидкість: " << CRSF_BAUD << " baud (CRSF кастомна)" << std::endl;
    std::cout << "  Відео: " << RTSP_URL << std::endl;
    std::cout << "  Покращення: PySerial-подібні налаштування" << std::endl << std::endl;
    
    // Перевірка наявності портів
    struct stat buffer;
    if (stat(RX_PORT.c_str(), &buffer) != 0) {
        std::cout << "❌ " << RX_PORT << " не знайдено!" << std::endl;
        return 1;
    }
    if (stat(FC_PORT.c_str(), &buffer) != 0) {
        std::cout << "❌ " << FC_PORT << " не знайдено!" << std::endl;
        return 1;
    }
    
    // Створення та запуск асинхронного bridge
    AsyncCrsfBridge bridge;
    global_bridge = &bridge;
    
    // Налаштування callback для помилок
    bridge.set_error_callback([](const std::string& error) {
        std::cout << "🔥 Bridge Error: " << error << std::endl;
    });
    
    if (!bridge.start()) {
        std::cout << "❌ Не вдалося запустити bridge" << std::endl;
        return 1;
    }
    
    // Запуск відеоплеєра
    SimpleVideoPlayer player;
    global_player = &player;
    
    if (!player.start(RTSP_URL)) {
        std::cout << "❌ Не вдалося запустити відео" << std::endl;
        bridge.stop();
        return 1;
    }
    
    std::cout << "✅ Система працює на швидкості " << bridge.get_baud() << " baud!" << std::endl;
    std::cout << "📊 Статистика кожні 5 секунд" << std::endl;
    std::cout << "🎯 Асинхронна обробка з Boost.Asio" << std::endl;
    std::cout << "🐍 PySerial-подібні налаштування портів" << std::endl;
    std::cout << "Натисніть Ctrl+C для зупинки" << std::endl << std::endl;
    
    // Головний цикл (GStreamer)
    player.run();
    
    // Очищення
    bridge.stop();
    player.stop();
    
    return 0;