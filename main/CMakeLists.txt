set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../components)

idf_component_register(SRCS "main.cpp"
                            "src/voltage.cpp"
                            "src/telemetry.cpp"
                            "src/sdlog.cpp"
                            "src/bmp280.cpp"
                            "src/ina219.cpp"
                       INCLUDE_DIRS ".")