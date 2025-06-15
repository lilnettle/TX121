# Makefile for Clean Video + CRSF Bridge (Native compilation on Radxa Zero 3W)

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -pthread

# ARM64 optimizations for Radxa Zero 3W (RK3566)
CXXFLAGS += -march=armv8-a -mtune=cortex-a55 -mfix-cortex-a53-835769 -mfix-cortex-a53-843419

# Program name
TARGET = bridge_radxa

# Source file
SOURCE = bridge_radxa.cpp

# GStreamer and GLib packages
GSTREAMER_PACKAGES = gstreamer-1.0 gstreamer-base-1.0
GLIB_PACKAGES = glib-2.0 gobject-2.0

# Get compile and link flags
GSTREAMER_CFLAGS = $(shell pkg-config --cflags $(GSTREAMER_PACKAGES))
GSTREAMER_LIBS = $(shell pkg-config --libs $(GSTREAMER_PACKAGES))
GLIB_CFLAGS = $(shell pkg-config --cflags $(GLIB_PACKAGES))
GLIB_LIBS = $(shell pkg-config --libs $(GLIB_PACKAGES))

# Combine all flags
ALL_CFLAGS = $(CXXFLAGS) $(GSTREAMER_CFLAGS) $(GLIB_CFLAGS)
ALL_LIBS = $(GSTREAMER_LIBS) $(GLIB_LIBS)

# Default target
all: $(TARGET)

# Build the main target
$(TARGET): $(SOURCE)
	@echo "🔨 Compiling $(TARGET) on Radxa Zero 3W..."
	$(CXX) $(ALL_CFLAGS) -o $(TARGET) $(SOURCE) $(ALL_LIBS)
	@echo "✅ Build complete: $(TARGET)"
	@echo "📏 Binary size: $$(du -h $(TARGET) | cut -f1)"

# Clean build artifacts
clean:
	@echo "🧹 Cleaning..."
	rm -f $(TARGET)
	@echo "✅ Clean complete"

# Install system dependencies on Radxa Zero 3W
install-deps:
	@echo "📦 Installing dependencies on Radxa Zero 3W..."
	sudo apt update
	sudo apt install -y \
		build-essential \
		pkg-config \
		libgstreamer1.0-dev \
		libgstreamer-plugins-base1.0-dev \
		libgstreamer-plugins-bad1.0-dev \
		libgstreamer-plugins-good1.0-dev \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-ugly \
		gstreamer1.0-libav \
		gstreamer1.0-tools \
		libglib2.0-dev
	@echo "✅ Dependencies installed"

# Install Rockchip MPP support (if available)
install-mpp:
	@echo "🚀 Installing Rockchip MPP support..."
	-sudo apt install -y \
		gstreamer1.0-rockchip1 \
		librockchip-mpp1 \
		librockchip-mpp-dev
	@echo "ℹ️  MPP installation complete (may not be available on all images)"

# Check if all dependencies are available
check-deps:
	@echo "🔍 Checking dependencies..."
	@pkg-config --exists $(GSTREAMER_PACKAGES) || (echo "❌ GStreamer packages not found" && exit 1)
	@pkg-config --exists $(GLIB_PACKAGES) || (echo "❌ GLib packages not found" && exit 1)
	@echo "✅ All dependencies found"

# Show build information
info:
	@echo "📋 Build Information:"
	@echo "Target: $(TARGET)"
	@echo "Compiler: $(CXX)"
	@echo "Platform: $$(uname -m)"
	@echo "OS: $$(lsb_release -d 2>/dev/null | cut -f2 || echo 'Unknown')"
	@echo "CPU: $$(nproc) cores"
	@echo "C++ Flags: $(CXXFLAGS)"
	@echo "GStreamer: $$(pkg-config --modversion gstreamer-1.0 2>/dev/null || echo 'Not found')"

# Quick build and run
run: $(TARGET)
	@echo "🚀 Running $(TARGET)..."
	sudo ./$(TARGET)

# Run without bridge (video only)
run-video-only: $(TARGET)
	@echo "🚀 Running video-only mode..."
	sudo ./$(TARGET) --no-bridge

# Debug build
debug: CXXFLAGS += -g -DDEBUG -O0
debug: clean $(TARGET)
	@echo "🐛 Debug build complete"

# Release build (optimized)
release: CXXFLAGS += -O3 -DNDEBUG -flto
release: clean $(TARGET)
	@echo "🚀 Release build complete"

# Test compilation without linking
compile-test:
	@echo "🧪 Testing compilation..."
	$(CXX) $(ALL_CFLAGS) -c $(SOURCE) -o test.o
	@echo "✅ Compilation test passed"
	rm -f test.o

# System diagnostics
diag:
	@echo "🔧 System Diagnostics:"
	@echo "Platform: $$(uname -a)"
	@echo "CPU Info:"
	@cat /proc/cpuinfo | grep -E "(processor|model name|cpu MHz)" | head -12
	@echo ""
	@echo "Memory:"
	@free -h
	@echo ""
	@echo "Temperature:"
	@cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | head -3 | while read temp; do echo "$$((temp/1000))°C"; done || echo "Temperature monitoring not available"
	@echo ""
	@echo "USB Serial Devices:"
	@ls -la /dev/ttyUSB* 2>/dev/null || echo "No USB serial devices found"
	@ls -la /dev/ttyACM* 2>/dev/null || echo "No ACM serial devices found"
	@echo ""
	@echo "Video Devices:"
	@ls -la /dev/video* 2>/dev/null || echo "No video devices found"

# Performance test
perf-test: $(TARGET)
	@echo "⚡ Performance test..."
	@echo "Compilation time test (3 runs):"
	@for i in 1 2 3; do \
		echo -n "Run $$i: "; \
		time $(MAKE) clean > /dev/null 2>&1 && time $(MAKE) $(TARGET) > /dev/null 2>&1; \
	done

# Create systemd service
install-service: $(TARGET)
	@echo "🔧 Installing systemd service..."
	@echo "[Unit]" > clean-video-bridge.service
	@echo "Description=Clean Video Bridge" >> clean-video-bridge.service
	@echo "After=network.target" >> clean-video-bridge.service
	@echo "" >> clean-video-bridge.service
	@echo "[Service]" >> clean-video-bridge.service
	@echo "Type=simple" >> clean-video-bridge.service
	@echo "User=root" >> clean-video-bridge.service
	@echo "WorkingDirectory=$$(pwd)" >> clean-video-bridge.service
	@echo "ExecStart=$$(pwd)/$(TARGET)" >> clean-video-bridge.service
	@echo "Restart=always" >> clean-video-bridge.service
	@echo "RestartSec=5" >> clean-video-bridge.service
	@echo "" >> clean-video-bridge.service
	@echo "[Install]" >> clean-video-bridge.service
	@echo "WantedBy=multi-user.target" >> clean-video-bridge.service
	sudo cp clean-video-bridge.service /etc/systemd/system/
	sudo systemctl daemon-reload
	@echo "✅ Service installed. Use:"
	@echo "  sudo systemctl enable clean-video-bridge"
	@echo "  sudo systemctl start clean-video-bridge"

# Remove systemd service
remove-service:
	@echo "🗑️  Removing systemd service..."
	sudo systemctl stop clean-video-bridge 2>/dev/null || true
	sudo systemctl disable clean-video-bridge 2>/dev/null || true
	sudo rm -f /etc/systemd/system/clean-video-bridge.service
	sudo systemctl daemon-reload
	rm -f clean-video-bridge.service
	@echo "✅ Service removed"

# Full setup from scratch
setup: install-deps install-mpp check-deps $(TARGET)
	@echo "🎉 Full setup complete!"
	@echo "Ready to run: sudo ./$(TARGET)"

# Help
help:
	@echo "🎬 Clean Video + CRSF Bridge - Radxa Zero 3W Native Build"
	@echo ""
	@echo "Quick Start:"
	@echo "  make setup           - Install dependencies and build"
	@echo "  make run             - Build and run with bridge"
	@echo "  make run-video-only  - Build and run video only"
	@echo ""
	@echo "Build Targets:"
	@echo "  make                 - Build program (default)"
	@echo "  make debug           - Build debug version"
	@echo "  make release         - Build optimized version"
	@echo "  make clean           - Remove build files"
	@echo ""
	@echo "Dependencies:"
	@echo "  make install-deps    - Install required packages"
	@echo "  make install-mpp     - Install Rockchip MPP support"
	@echo "  make check-deps      - Verify dependencies"
	@echo ""
	@echo "System:"
	@echo "  make info            - Show build information"
	@echo "  make diag            - System diagnostics"
	@echo "  make perf-test       - Performance test"
	@echo ""
	@echo "Service:"
	@echo "  make install-service - Install systemd service"
	@echo "  make remove-service  - Remove systemd service"
	@echo ""
	@echo "Configuration (hardcoded):"
	@echo "  RTSP: rtsp://root:12345@192.168.0.100:554/stream1"
	@echo "  Video: 1280x720@30fps"
	@echo "  CRSF: 420000 baud"
	@echo "  Ports: /dev/ttyUSB1 → /dev/ttyUSB0"

# Mark targets as phony
.PHONY: all clean install-deps install-mpp check-deps info run run-video-only debug release compile-test diag perf-test install-service remove-service setup help