#!/bin/bash

# =============================================================================
# Simple HDMI OSD Launcher
# Запуск з фіксованою IP камерою rtsp://root:12345@192.168.0.100:554/stream1
# =============================================================================

# Кольори
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

# Конфігурація
RTSP_URL="rtsp://root:12345@192.168.0.100:554/stream1"
CRSF_PORT="/dev/ttyUSB0"
CRSF_BAUD="420000"
RESOLUTION="1920x1080"
FRAMERATE="30"

echo -e "${CYAN}🎬 HDMI OSD Launcher${NC}"
echo "Камера: $RTSP_URL"
echo "CRSF: $CRSF_PORT @ $CRSF_BAUD"
echo "Вихід: HDMI $RESOLUTION @ ${FRAMERATE}fps"
echo

# Перевірити файл програми
if [ ! -f "hdmi_osd_player.py" ]; then
    echo -e "${RED}❌ Файл hdmi_osd_player.py не знайдений${NC}"
    exit 1
fi

# Перевірити CRSF порт
if [ ! -e "$CRSF_PORT" ]; then
    echo -e "${YELLOW}⚠️ CRSF порт $CRSF_PORT не знайдений${NC}"
    
    # Спробувати знайти інші порти
    OTHER_PORTS=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -3)
    if [ -n "$OTHER_PORTS" ]; then
        echo -e "${BLUE}Знайдені порти:${NC}"
        for port in $OTHER_PORTS; do
            echo "  📡 $port"
        done
        
        read -p "Використати інший порт? Введіть шлях або Enter для $CRSF_PORT: " NEW_PORT
        if [ -n "$NEW_PORT" ]; then
            CRSF_PORT="$NEW_PORT"
        fi
    else
        echo -e "${YELLOW}Продовжити без CRSF телеметрії?${NC}"
        read -p "(y/n): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
        CRSF_PORT=""
    fi
fi

# Перевірити доступ до камери
echo -e "${BLUE}🔍 Перевірка камери...${NC}"
if timeout 5s ffprobe "$RTSP_URL" >/dev/null 2>&1; then
    echo -e "${GREEN}✅ Камера доступна${NC}"
elif command -v ffprobe >/dev/null; then
    echo -e "${YELLOW}⚠️ Камера може бути недоступна, але спробуємо підключитися${NC}"
else
    echo -e "${YELLOW}⚠️ ffprobe не встановлений, пропускаємо перевірку камери${NC}"
fi

# Сформувати команду
CMD="python3 hdmi_osd_player.py -i \"$RTSP_URL\" -r $RESOLUTION -f $FRAMERATE"

if [ -n "$CRSF_PORT" ]; then
    CMD="$CMD -p $CRSF_PORT -b $CRSF_BAUD"
fi

echo
echo -e "${GREEN}Команда запуску:${NC}"
echo "$CMD"
echo

# Показати режими
echo -e "${YELLOW}Режими запуску:${NC}"
echo "1) Повноекранний (рекомендовано)"
echo "2) Віконний (для тестування)"
echo "3) Показати додаткові опції"
echo "4) Запустити як є"

read -p "Ваш вибір (1-4): " mode_choice

case $mode_choice in
    1)
        echo -e "${BLUE}🖥️ Повноекранний режим${NC}"
        ;;
    2)
        echo -e "${BLUE}🪟 Віконний режим${NC}"
        CMD="$CMD --windowed"
        ;;
    3)
        echo -e "${BLUE}⚙️ Додаткові опції:${NC}"
        echo
        echo "Роздільність:"
        echo "  a) 1280x720 (HD)"
        echo "  b) 1920x1080 (Full HD)"
        echo "  c) 3840x2160 (4K)"
        read -p "Оберіть (a-c) або Enter для поточної: " res_choice
        
        case $res_choice in
            a) CMD=$(echo "$CMD" | sed "s/$RESOLUTION/1280x720/") ;;
            c) CMD=$(echo "$CMD" | sed "s/$RESOLUTION/3840x2160/") ;;
        esac
        
        echo "Кадрова частота:"
        echo "  a) 30 fps (стандарт)"
        echo "  b) 60 fps (плавність)"
        read -p "Оберіть (a-b) або Enter для поточної: " fps_choice
        
        case $fps_choice in
            b) CMD=$(echo "$CMD" | sed "s/$FRAMERATE/60/") ;;
        esac
        
        read -p "Віконний режим? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            CMD="$CMD --windowed"
        fi
        ;;
    4)
        echo -e "${BLUE}🚀 Запуск з поточними налаштуваннями${NC}"
        ;;
    *)
        echo -e "${RED}❌ Невірний вибір, використовую повноекранний режим${NC}"
        ;;
esac

echo
echo -e "${GREEN}Фінальна команда:${NC}"
echo "$CMD"
echo

read -p "Запустити? (y/n): " -n 1 -r
echo

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${CYAN}🎬 Запуск HDMI OSD...${NC}"
    echo -e "${YELLOW}Натисніть Ctrl+C для зупинки${NC}"
    echo
    sleep 2
    
    # Запустити команду
    eval $CMD
else
    echo -e "${YELLOW}Запуск скасовано${NC}"
fi