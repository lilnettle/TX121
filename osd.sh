#!/bin/bash

# =============================================================================
# GStreamer Plugins Installation Script
# Мінімальне встановлення тільки необхідних GStreamer плагінів
# =============================================================================

set -e  # Зупинити при помилці

# Кольори для виводу
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m' # No Color

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Визначити дистрибутив
detect_distro() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        DISTRO=$ID
    else
        print_error "Cannot detect Linux distribution"
        exit 1
    fi
    print_step "Detected: $PRETTY_NAME"
}

# Встановити GStreamer та плагіни
install_gstreamer() {
    print_step "Installing GStreamer and essential plugins..."
    
    case $DISTRO in
        ubuntu|debian)
            sudo apt update
            sudo apt install -y \
                gstreamer1.0-tools \
                gstreamer1.0-plugins-base \
                gstreamer1.0-plugins-good \
                gstreamer1.0-plugins-bad \
                gstreamer1.0-plugins-ugly \
                gstreamer1.0-libav \
                gstreamer1.0-x \
                gstreamer1.0-alsa \
                python3-gi \
                python3-gi-cairo \
                gir1.2-gstreamer-1.0 \
                gir1.2-gst-plugins-base-1.0 \
                gir1.2-gst-rtsp-server-1.0 \
                python3-serial
            ;;
        fedora)
            sudo dnf install -y \
                gstreamer1-tools \
                gstreamer1-plugins-base \
                gstreamer1-plugins-good \
                gstreamer1-plugins-bad-free \
                gstreamer1-plugins-ugly-free \
                gstreamer1-libav \
                python3-gobject \
                python3-cairo \
                gstreamer1-rtsp-server \
                python3-pyserial
            ;;
        arch|manjaro)
            sudo pacman -S --noconfirm \
                gstreamer \
                gst-plugins-base \
                gst-plugins-good \
                gst-plugins-bad \
                gst-plugins-ugly \
                gst-libav \
                gst-rtsp-server \
                python-gobject \
                python-cairo \
                python-pyserial
            ;;
        *)
            print_error "Unsupported distribution: $DISTRO"
            exit 1
            ;;
    esac
    
    print_success "GStreamer plugins installed"
}

# Тестування плагінів
test_plugins() {
    print_step "Testing required GStreamer plugins..."
    
    # Список необхідних плагінів
    REQUIRED_PLUGINS=(
        "rtspsrc"
        "rtph264depay" 
        "avdec_h264"
        "videoconvert"
        "textoverlay"
        "cairooverlay"
        "videoscale"
        "x264enc"
        "rtph264pay"
        "udpsink"
        "autovideosink"
        "videotestsrc"
    )
    
    MISSING_PLUGINS=()
    
    for plugin in "${REQUIRED_PLUGINS[@]}"; do
        if gst-inspect-1.0 $plugin >/dev/null 2>&1; then
            echo -e "  ✅ $plugin"
        else
            echo -e "  ❌ $plugin"
            MISSING_PLUGINS+=($plugin)
        fi
    done
    
    if [ ${#MISSING_PLUGINS[@]} -eq 0 ]; then
        print_success "All required plugins available"
        return 0
    else
        print_error "Missing plugins: ${MISSING_PLUGINS[*]}"
        return 1
    fi
}

# Тестовий pipeline
test_pipeline() {
    print_step "Testing basic pipeline..."
    
    # Простий тест pipeline
    timeout 3s gst-launch-1.0 \
        videotestsrc num-buffers=30 ! \
        textoverlay text="TEST" ! \
        videoconvert ! \
        fakesink >/dev/null 2>&1
    
    if [ $? -eq 0 ] || [ $? -eq 124 ]; then  # 124 = timeout (успіх)
        print_success "Pipeline test passed"
        return 0
    else
        print_error "Pipeline test failed"
        return 1
    fi
}

# Показати доступні плагіни
show_available_plugins() {
    echo -e "\n${CYAN}Available GStreamer plugins:${NC}"
    echo -e "${YELLOW}Video sources:${NC}"
    gst-inspect-1.0 | grep -E "(videotestsrc|rtspsrc|v4l2src)" | head -5
    
    echo -e "${YELLOW}Video sinks:${NC}"
    gst-inspect-1.0 | grep -E "(autovideosink|ximagesink|xvimagesink)" | head -5
    
    echo -e "${YELLOW}Overlays:${NC}"
    gst-inspect-1.0 | grep -E "(textoverlay|cairooverlay|clockoverlay)" | head -5
    
    echo -e "${YELLOW}Encoders:${NC}"
    gst-inspect-1.0 | grep -E "(x264enc|openh264enc|avenc_h264)" | head -5
}

# Головна функція
main() {
    echo -e "${CYAN}GStreamer Plugins Installer${NC}"
    echo "Installing only essential GStreamer plugins for CRSF OSD"
    echo
    
    detect_distro
    install_gstreamer
    
    echo
    test_plugins
    test_pipeline
    
    echo
    show_available_plugins
    
    echo
    print_success "Installation complete!"
    echo -e "${YELLOW}Test command:${NC}"
    echo "gst-launch-1.0 videotestsrc ! textoverlay text='Hello World' ! autovideosink"
}

main "$@"