
file(GLOB_RECURSE radiolib_sources "src/*.c" "src/*.cpp")

idf_component_register(SRCS ${radiolib_sources} 
                       INCLUDE_DIRS src
                       REQUIRES espressif__arduino-esp32)
