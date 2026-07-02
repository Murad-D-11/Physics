cmake_minimum_required(VERSION 3.10)
project(Physics)

set(CMAKE_CXX_STANDARD 17) # minimum c++ version
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(${PROJECT_NAME} src/app/main.cpp src/renderer/camera.cpp src/renderer/cube.cpp src/renderer/render.cpp src/renderer/glad.c src/physics/physicssolver.cpp src/physics/aabb.cpp src/physics/collision.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include) # locate include
target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/lib) # locate lib
target_link_libraries(${PROJECT_NAME} PRIVATE glfw3dll) # locate .dll

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_SOURCE_DIR}/glfw3.dll" "$<TARGET_FILE_DIR:${PROJECT_NAME}>")