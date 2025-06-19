# Makefile для Async CRSF Bridge з Boost.Asio

CXX = g++
CXXFLAGS = -std=c++11 -Wall -O2 -pthread
TARGET = bridge_radxa
SOURCE = bridge_radxa.cpp

# Boost бібліотеки
BOOST_LIBS = -lboost_system -lboost_thread

# GStreamer бібліотеки
GSTREAMER_CFLAGS = $(shell pkg-config --cflags gstreamer-1.0)
GSTREAMER_LIBS = $(shell pkg-config --libs gstreamer-1.0)

# GLib бібліотеки
GLIB_CFLAGS = $(shell pkg-config --cflags glib-2.0)
GLIB_LIBS = $(shell pkg-config --libs glib-2.0)

# Всі флаги разом
ALL_CFLAGS = $(CXXFLAGS) $(GSTREAMER_CFLAGS) $(GLIB_CFLAGS)
ALL_LIBS = $(BOOST_LIBS) $(GSTREAMER_LIBS) $(GLIB_LIBS)

# Основна ціль
$(TARGET): $(SOURCE)
	$(CXX) $(ALL_CFLAGS) -o $(TARGET) $(SOURCE) $(ALL_LIBS)

# Встановлення залежностей в Debian/Ubuntu
install-deps:
	sudo apt-get update
	sudo apt-get install -y \
		libboost-all-dev \
		libgstreamer1.0-dev \
		libgstreamer-plugins-base1.0-dev \
		libglib2.0-dev \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-ugly \
		gstreamer1.0-libav \
		pkg-config

# Перевірка залежностей
check-deps:
	@echo "Перевірка Boost.Asio..."
	@pkg-config --exists --print-errors "boost" || echo "❌ Boost не знайдено"
	@echo "Перевірка GStreamer..."
	@pkg-config --exists --print-errors "gstreamer-1.0" || echo "❌ GStreamer не знайдено"
	@echo "Перевірка GLib..."
	@pkg-config --exists --print-errors "glib-2.0" || echo "❌ GLib не знайдено"
	@echo "✅ Перевірка завершена"

# Очищення
clean:
	rm -f $(TARGET)

# Debug версія
debug: CXXFLAGS += -g -DDEBUG
debug: $(TARGET)

# Запуск з правами root (для доступу до серійних портів)
run: $(TARGET)
	sudo ./$(TARGET)

# Перевірка портів
check-ports:
	@echo "Перевірка серійних портів..."
	@ls -la /dev/ttyUSB* 2>/dev/null || echo "❌ USB порти не знайдено"
	@ls -la /dev/ttyACM* 2>/dev/null || echo "ℹ️ ACM порти не знайдено"

# Налаштування прав доступу до портів
setup-permissions:
	sudo usermod -a -G dialout $(USER)
	@echo "⚠️ Вийдіть і увійдіть знову для застосування змін"

# Показати інформацію про серійні порти
port-info:
	@echo "=== Серійні порти ==="
	@for port in /dev/ttyUSB* /dev/ttyACM*; do \
		if [ -e "$$port" ]; then \
			echo "Порт: $$port"; \
			udevadm info --name=$$port --query=property | grep -E "(ID_VENDOR|ID_MODEL|ID_SERIAL)" || true; \
			echo "---"; \
		fi \
	done

# Все разом: встановлення, компіляція, запуск
all: install-deps $(TARGET)
	@echo "✅ Готово! Запустіть 'make run' для старту"

.PHONY: install-deps check-deps clean debug run check-ports setup-permissions port-info all