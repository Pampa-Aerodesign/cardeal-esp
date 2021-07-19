set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../components)

idf_component_register(SRCS "main.cpp" "src/VoltageSensor.cpp"
                    INCLUDE_DIRS ".")