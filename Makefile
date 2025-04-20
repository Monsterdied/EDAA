

# Build and run
build:
	@mkdir -p build && \
	cd build && \
	cmake .. && \
	make
run:
	@cd build && \
	./EDAA_Project

# Clean build artifacts
clean:
	@rm -rf build
# Combined build+run

all: clean build run

.PHONY: run clean