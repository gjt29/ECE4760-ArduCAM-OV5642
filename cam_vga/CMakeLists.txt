add_executable(2040camera)

add_subdirectory(ArduCAM)
include_directories(ArduCAM)

# must match with pio filename and executable name from above
pico_generate_pio_header(2040camera ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(2040camera ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(2040camera ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)
pico_generate_pio_header(2040camera ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

# must match with executable name and source file names
target_sources(2040camera PRIVATE 2040camera.cpp vga_graphics.c)

# must match with executable name
target_link_libraries(2040camera PRIVATE pico_stdlib hardware_pio hardware_dma hardware_i2c ArduCAM hardware_spi hardware_irq)

# must match with executable name
pico_add_extra_outputs(2040camera)
