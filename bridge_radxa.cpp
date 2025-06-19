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

// Асинхронний CRSF Bridge з Boost.Asio
class AsyncCrsfBridge {
private:
    io_context io_ctx;
    serial_port rx_port, fc_port;
    deadline_timer stats_timer;
    std::thread io_thread;
    
    // Буфери для асинхронного читання
    static constexpr size_t BUFFER_SIZE = 1024;
    std::vector<uint8_t> rx_buffer, fc_buffer;
    
    // Статистика
    std::atomic<long> rx_to_fc_bytes{0}, fc_to_rx_bytes{0}, errors{0};
    std::atomic<bool> running{false};
    
    // Callbacks для обробки помилок
    std::function<void(const std::string&)> error_callback;

    // Налаштування кастомної швидкості 420000 baud
    bool setup_custom_baud_420000(serial_port& port) {
        try {
            // Спочатку встановлюємо базові параметри порту
            port.set_option(serial_port_base::baud_rate(38400)); // Базова швидкість
            port.set_option(serial_port_base::character_size(8));
            port.set_option(serial_port_base::parity(serial_port_base::parity::none));
            port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
            
            // Отримуємо нативний handle для налаштування кастомної швидкості
            int fd = port.native_handle();
            
            struct serial_struct ss;
            if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
                ss.flags = (ss.flags & ~0x0030) | 0x0010;  // ASYNC_SPD_CUST
                ss.custom_divisor = ss.baud_base / 420000;
                
                if (ioctl(fd, TIOCSSERIAL, &ss) == 0) {
                    std::cout << "✅ Custom baud 420000 встановлено успішно" << std::endl;
                    return true;
                } else {
                    std::cout << "❌ Не вдалося встановити кастомну швидкість 420000" << std::endl;
                    return false;
                }
            }
            return false;
            
        } catch (boost::system::system_error& e) {
            std::cout << "❌ Помилка налаштування порту: " << e.what() << std::endl;
            return false;
        }
    }

public:
    AsyncCrsfBridge() 
        : rx_port(io_ctx), fc_port(io_ctx), stats_timer(io_ctx),
          rx_buffer(BUFFER_SIZE), fc_buffer(BUFFER_SIZE) {}
    
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
            std::cout << "🔌 Підключення на швидкості " << CRSF_BAUD << " baud..." << std::endl;
            
            // Відкриття портів
            rx_port.open(RX_PORT);
            fc_port.open(FC_PORT);
            
            // Налаштування кастомної швидкості для обох портів
            if (!setup_custom_baud_420000(rx_port) || !setup_custom_baud_420000(fc_port)) {
                std::cout << "❌ Не вдалося налаштувати кастомну швидкість" << std::endl;
                return false;
            }
            
            std::cout << "✅ Підключено на швидкості " << CRSF_BAUD << " baud" << std::endl;
            std::cout << "📡 Bridge: " << RX_PORT << " ↔ " << FC_PORT << std::endl;
            
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
        if (rx_port.is_open()) {
            rx_port.cancel(ec);
            rx_port.close(ec);
        }
        if (fc_port.is_open()) {
            fc_port.cancel(ec);
            fc_port.close(ec);
        }
        
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
        rx_port.async_read_some(
            buffer(rx_buffer),
            boost::bind(&AsyncCrsfBridge::handle_rx_read, this,
                placeholders::error,
                placeholders::bytes_transferred));
    }
    
    void handle_rx_read(const boost::system::error_code& error, size_t bytes_transferred) {
        if (!running) return;
        
        if (!error && bytes_transferred > 0) {
            // Передача даних від RX до FC (USB1 → USB0)
            async_write(fc_port,
                buffer(rx_buffer, bytes_transferred),
                boost::bind(&AsyncCrsfBridge::handle_rx_write, this,
                    placeholders::error,
                    placeholders::bytes_transferred,
                    bytes_transferred));
            
        } else if (error) {
            handle_error("RX read", error);
        }
        
        // Продовження читання
        if (running && rx_port.is_open()) {
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
        fc_port.async_read_some(
            buffer(fc_buffer),
            boost::bind(&AsyncCrsfBridge::handle_fc_read, this,
                placeholders::error,
                placeholders::bytes_transferred));
    }
    
    void handle_fc_read(const boost::system::error_code& error, size_t bytes_transferred) {
        if (!running) return;
        
        if (!error && bytes_transferred > 0) {
            // Передача телеметрії від FC до RX (USB0 → USB1)
            async_write(rx_port,
                buffer(fc_buffer, bytes_transferred),
                boost::bind(&AsyncCrsfBridge::handle_fc_write, this,
                    placeholders::error,
                    placeholders::bytes_transferred,
                    bytes_transferred));
            
        } else if (error) {
            handle_error("FC read", error);
        }
        
        // Продовження читання
        if (running && fc_port.is_open()) {
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
            boost::bind(&AsyncCrsfBridge::handle_stats_timer, this,
                placeholders::error));
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
    
    std::cout << "🌉 ASYNC CRSF BRIDGE (Boost.Asio)" << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << "Конфігурація:" << std::endl;
    std::cout << "  RX Input:  " << RX_PORT << std::endl;
    std::cout << "  FC Output: " << FC_PORT << std::endl;
    std::cout << "  Напрямок: USB1 ↔ USB0 (двосторонній)" << std::endl;
    std::cout << "  Швидкість: " << CRSF_BAUD << " baud (CRSF кастомна)" << std::endl;
    std::cout << "  Відео: " << RTSP_URL << std::endl << std::endl;
    
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
    std::cout << "Натисніть Ctrl+C для зупинки" << std::endl << std::endl;
    
    // Головний цикл (GStreamer)
    player.run();
    
    // Очищення
    bridge.stop();
    player.stop();
    
    return 0;
}