project(tile-server)


find_package(Boost REQUIRED COMPONENTS coroutine)
add_executable(tile-server src/main.cpp)
target_link_libraries(tile-server PRIVATE Boost::headers Boost::coroutine)
if(MSVC)
    target_compile_options(tile-server PRIVATE /bigobj)
endif()

set_target_properties(tile-server PROPERTIES 
    CXX_EXTENSIONS OFF 
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

file(COPY websrc DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
