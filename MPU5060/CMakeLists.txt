# MPU6050/CMakeLists.txt

# Cria uma biblioteca estática chamada 'mpu6050' a partir do arquivo 'mpu6050.c'
add_library(mpu6050 STATIC mpu6050.c)

# Define as propriedades da biblioteca
target_include_directories(mpu6050 PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(mpu6050 PUBLIC pico_stdlib hardware_i2c hardware_gpio)

# Instala a biblioteca e os arquivos de cabeçalho
install(TARGETS mpu6050 DESTINATION lib)
install(FILES mpu6050.h DESTINATION include)
