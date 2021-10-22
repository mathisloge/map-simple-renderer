project(renderer)

find_package(spdlog CONFIG REQUIRED)
find_package(mapnik CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread timer)
mapnik_find_plugin_dir(MAPNIK_PLUGINS_DIR)

add_executable(renderer src/main.cpp src/bounded_buffer.hpp)
configure_file(version.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/version.hpp)
target_include_directories(renderer PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(renderer PRIVATE 
    mapnik::mapnik
    Boost::headers
    Boost::thread
    Boost::timer
    spdlog::spdlog
)
message(STATUS "plugins path: ${MAPNIK_PLUGINS_DIR}")

set_target_properties(renderer PROPERTIES 
    CXX_EXTENSIONS OFF 
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)
file(COPY fonts DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
