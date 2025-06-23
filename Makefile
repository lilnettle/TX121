# Makefile для RADXA CRSF Bridge з Feedback
# Компілятор та основні флаги
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -pthread
DEBUG_FLAGS = -g -DDEBUG -fsanitize=address
TARGET = bridge_radxa
SOURCE = bridge_radxa.cpp

# GStreamer бібліотеки
GSTREAMER_CFLAGS = $(shell pkg-config --cflags gstreamer-1.0 2>/dev/null)
GSTREAMER_LIBS = $(shell pkg-config --libs gstreamer-1.0 2>/dev/null)

# GLib бібліотеки  
GLIB_CFLAGS = $(shell pkg-config --cflags glib-2.0 2>/dev/null)
GLIB_LIBS = $(shell pkg-config --libs glib-2.0 2>/dev/null)

# Всі флаги разом
ALL_CFLAGS = $(CXXFLAGS) $(GSTREAMER_CFLAGS) $(GLIB_CFLAGS)
ALL_LIBS = $(GSTREAMER_LIBS) $(GLIB_LIBS)

# Кольори для виводу
RED = \033[0;31m
GREEN = \033[0;32m
YELLOW = \033[0;33m
BLUE = \033[0;34m
NC = \033[0m # No Color

# Основна ціль
$(TARGET): $(SOURCE)
	@echo "$(BLUE)🔨 Компіляція $(TARGET)...$(NC)"
	$(CXX) $(ALL_CFLAGS) -o $(TARGET) $(SOURCE) $(ALL_LIBS)
	@echo "$(GREEN)✅ Компіляція успішна!$(NC)"

# Встановлення залежностей для різних дистрибутивів
install-deps:
	@echo "$(BLUE)📦 Встановлення залежностей...$(NC)"
	@if command -v apt-get >/dev/null 2>&1; then \
		echo "$(YELLOW)Використовується APT (Debian/Ubuntu)$(NC)"; \
		sudo apt-get update && \
		sudo apt-get install -y \
			build-essential \
			pkg-config \
			libgstreamer1.0-dev \
			libgstreamer-plugins-base1.0-dev \
			libglib2.0-dev \
			gstreamer1.0-plugins-good \
			gstreamer1.0-plugins-bad \
			gstreamer1.0-plugins-ugly \
			gstreamer1.0-libav \
			gstreamer1.0-tools; \
	elif command -v yum >/dev/null 2>&1; then \
		echo "$(YELLOW)Використовується YUM (CentOS/RHEL)$(NC)"; \
		sudo yum groupinstall -y "Development Tools" && \
		sudo yum install -y \
			pkg-config \
			gstreamer1-devel \
			gstreamer1-plugins-base-devel \
			glib2-devel \
			gstreamer1-plugins-good \
			gstreamer1-plugins-bad \
			gstreamer1-plugins-ugly; \
	elif command -v pacman >/dev/null 2>&1; then \
		echo "$(YELLOW)Використовується Pacman (Arch Linux)$(NC)"; \
		sudo pacman -S --needed \
			base-devel \
			pkg-config \
			gstreamer \
			gst-plugins-base \
			gst-plugins-good \
			gst-plugins-bad \
			gst-plugins-ugly \
			glib2; \
	else \
		echo "$(RED)❌ Невідомий пакетний менеджер!$(NC)"; \
		echo "Встановіть вручну: build-essential, pkg-config, gstreamer1.0-dev, glib2.0-dev"; \
		exit 1; \
	fi
	@echo "$(GREEN)✅ Залежності встановлено!$(NC)"

# Перевірка залежностей
check-deps:
	@echo "$(BLUE)🔍 Перевірка залежностей...$(NC)"
	@echo -n "GCC/G++: "; $(CXX) --version | head -n1 || echo "$(RED)❌ Не знайдено$(NC)"
	@echo -n "pkg-config: "; pkg-config --version || echo "$(RED)❌ Не знайдено$(NC)"
	@echo -n "GStreamer: "; pkg-config --modversion gstreamer-1.0 2>/dev/null && echo "$(GREEN)✅$(NC)" || echo "$(RED)❌ Не знайдено$(NC)"
	@echo -n "GLib: "; pkg-config --modversion glib-2.0 2>/dev/null && echo "$(GREEN)✅$(NC)" || echo "$(RED)❌ Не знайдено$(NC)"
	@echo -n "C++17 підтримка: "; echo 'int main(){return 0;}' | $(CXX) -std=c++17 -x c++ - -o /tmp/cpp17test 2>/dev/null && echo "$(GREEN)✅$(NC)" || echo "$(RED)❌ Не підтримується$(NC)"
	@rm -f /tmp/cpp17test

# Перевірка серійних портів
check-ports:
	@echo "$(BLUE)🔌 Перевірка серійних портів...$(NC)"
	@echo "USB серійні порти:"
	@ls -la /dev/ttyUSB* 2>/dev/null | while read line; do echo "  $(GREEN)✅$(NC) $$line"; done || echo "  $(YELLOW)⚠️ USB порти не знайдено$(NC)"
	@echo "ACM серійні порти:"
	@ls -la /dev/ttyACM* 2>/dev/null | while read line; do echo "  $(GREEN)✅$(NC) $$line"; done || echo "  $(YELLOW)⚠️ ACM порти не знайдено$(NC)"
	@echo "UART порти (Pi/Radxa):"
	@ls -la /dev/ttyAMA* /dev/ttyS* 2>/dev/null | while read line; do echo "  $(GREEN)✅$(NC) $$line"; done || echo "  $(YELLOW)⚠️ UART порти не знайдено$(NC)"

# Детальна інформація про порти
port-info:
	@echo "$(BLUE)📋 Детальна інформація про серійні порти:$(NC)"
	@for port in /dev/ttyUSB* /dev/ttyACM* /dev/ttyAMA* /dev/ttyS*; do \
		if [ -e "$$port" ]; then \
			echo "$(YELLOW)📍 Порт: $$port$(NC)"; \
			if command -v udevadm >/dev/null 2>&1; then \
				udevadm info --name=$$port --query=property 2>/dev/null | grep -E "(ID_VENDOR|ID_MODEL|ID_SERIAL|DEVPATH)" | sed 's/^/  /' || true; \
			fi; \
			ls -la $$port | sed 's/^/  Права: /'; \
			echo ""; \
		fi \
	done

# Налаштування прав доступу
setup-permissions:
	@echo "$(BLUE)🔐 Налаштування прав доступу...$(NC)"
	sudo usermod -a -G dialout $(USER)
	sudo usermod -a -G tty $(USER)
	@echo "$(GREEN)✅ Користувача $(USER) додано до груп dialout та tty$(NC)"
	@echo "$(YELLOW)⚠️ Вийдіть і увійдіть знову (або перезавантажтесь) для застосування змін$(NC)"

# Debug версія з додатковими перевірками
debug: CXXFLAGS += $(DEBUG_FLAGS)
debug: clean $(TARGET)
	@echo "$(GREEN)✅ Debug версія скомпільована з AddressSanitizer$(NC)"

# Версія з оптимізацією
release: CXXFLAGS += -O3 -DNDEBUG -march=native
release: clean $(TARGET)
	@echo "$(GREEN)✅ Release версія з максимальною оптимізацією$(NC)"

# Запуск програми
run: $(TARGET)
	@echo "$(BLUE)🚀 Запуск $(TARGET)...$(NC)"
	@if [ "$$(id -u)" = "0" ]; then \
		./$(TARGET); \
	else \
		echo "$(YELLOW)⚠️ Потрібні права root для доступу до серійних портів$(NC)"; \
		sudo ./$(TARGET); \
	fi

# Запуск у debug режимі
run-debug: debug
	@echo "$(BLUE)🐛 Запуск debug версії...$(NC)"
	sudo ./$(TARGET)

# Тестування з фейковими даними
test: $(TARGET)
	@echo "$(BLUE)🧪 Тестовий запуск...$(NC)"
	@echo "$(YELLOW)Це буде запуск у тестовому режимі з логуванням$(NC)"
	sudo strace -e trace=network,file ./$(TARGET) || true

# Моніторинг портів у реальному часі
monitor-ports:
	@echo "$(BLUE)👁️ Моніторинг серійних портів (Ctrl+C для виходу)...$(NC)"
	@while true; do \
		clear; \
		echo "$(BLUE)📅 $$(date)$(NC)"; \
		echo "$(BLUE)🔌 Активні серійні порти:$(NC)"; \
		ls -la /dev/tty{USB,ACM,AMA,S}* 2>/dev/null || echo "  Порти не знайдено"; \
		echo ""; \
		echo "$(BLUE)💻 Процеси що використовують порти:$(NC)"; \
		lsof /dev/tty* 2>/dev/null | head -10 || echo "  Немає активних процесів"; \
		sleep 2; \
	done

# Логування системних повідомлень
watch-logs:
	@echo "$(BLUE)📜 Перегляд системних логів (Ctrl+C для виходу)...$(NC)"
	sudo dmesg -w | grep -i -E "(usb|serial|tty|cdc|acm)" --color=always

# Очищення
clean:
	@echo "$(BLUE)🧹 Очищення...$(NC)"
	rm -f $(TARGET)
	rm -f core dump.*
	@echo "$(GREEN)✅ Очищення завершено$(NC)"

# Показати довідку
help:
	@echo "$(BLUE)📖 RADXA CRSF Bridge - Довідка по командам:$(NC)"
	@echo ""
	@echo "$(YELLOW)📦 Встановлення та налаштування:$(NC)"
	@echo "  make install-deps    - Встановити всі залежності"
	@echo "  make check-deps      - Перевірити наявність залежностей"
	@echo "  make setup-permissions - Налаштувати права доступу до портів"
	@echo ""
	@echo "$(YELLOW)🔨 Компіляція:$(NC)"
	@echo "  make                 - Звичайна компіляція"
	@echo "  make debug          - Debug версія з AddressSanitizer"
	@echo "  make release        - Оптимізована версія"
	@echo ""
	@echo "$(YELLOW)🚀 Запуск:$(NC)"
	@echo "  make run            - Запустити програму"
	@echo "  make run-debug      - Запустити debug версію"
	@echo "  make test           - Тестовий запуск з strace"
	@echo ""
	@echo "$(YELLOW)🔍 Діагностика:$(NC)"
	@echo "  make check-ports    - Перевірити серійні порти"
	@echo "  make port-info      - Детальна інформація про порти"
	@echo "  make monitor-ports  - Моніторинг портів у реальному часі"
	@echo "  make watch-logs     - Перегляд системних логів"
	@echo ""
	@echo "$(YELLOW)🧹 Обслуговування:$(NC)"
	@echo "  make clean          - Очистити скомпільовані файли"
	@echo "  make help           - Показати цю довідку"

# Швидка установка всього необхідного
quick-setup: install-deps setup-permissions $(TARGET)
	@echo "$(GREEN)🎉 Швидке налаштування завершено!$(NC)"
	@echo "$(YELLOW)⚠️ Перезавантажтесь або перелогіньтесь для застосування прав доступу$(NC)"
	@echo "$(BLUE)Після цього запустіть: make run$(NC)"

# За замовчуванням - показати довідку
.DEFAULT_GOAL := help

# Позначити фальшиві цілі
.PHONY: install-deps check-deps check-ports port-info setup-permissions debug release run run-debug test monitor-ports watch-logs clean help quick-setup