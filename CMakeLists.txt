cmake_minimum_required(VERSION 3.10)
project(Physics)

set(CMAKE_CXX_STANDARD 17) # minimum c++ version
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to an optimized build. Without a build type, CMake configures with
# no optimization (-O0), which makes GLM + std containers roughly 20x slower.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_executable(${PROJECT_NAME} src/app/main.cpp src/renderer/camera.cpp src/renderer/cube.cpp src/renderer/sphere.cpp src/renderer/render.cpp src/renderer/glad.c src/physics/physicssolver.cpp src/physics/obb.cpp src/physics/collision.cpp src/renderer/ground.cpp)

# Always optimize this target. GLM + std containers run ~20x slower at -O0, and
# editor/CMake tooling tends to reconfigure to a Debug (-O0) build, which
# silently reintroduces the slowdown. Forcing -O2 on the target avoids that.
target_compile_options(${PROJECT_NAME} PRIVATE -O2)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include) # locate include
target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/lib) # locate lib
target_link_libraries(${PROJECT_NAME} PRIVATE glfw3dll) # locate .dll

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_SOURCE_DIR}/glfw3.dll" "$<TARGET_FILE_DIR:${PROJECT_NAME}>")

# ---------------------------------------------------------------------------
# Headless resting-contact audit harness (no OpenGL). Links only the physics
# translation units. Build with:  cmake --build build --target PhysicsTests
# ---------------------------------------------------------------------------
add_executable(PhysicsTests src/app/physics_tests.cpp src/physics/physicssolver.cpp src/physics/obb.cpp src/physics/collision.cpp)
target_compile_options(PhysicsTests PRIVATE -O2)
target_include_directories(PhysicsTests PRIVATE ${CMAKE_SOURCE_DIR}/include)

# ---------------------------------------------------------------------------
# Headless ML dataset generator (no OpenGL). Uses the rendering-independent
# Environment (src/ml) over the physics solver to emit supervised-learning CSV
# datasets. Build with:  cmake --build build --target DatasetGenerator
# ---------------------------------------------------------------------------
add_executable(DatasetGenerator src/app/dataset_main.cpp src/physics/physicssolver.cpp src/physics/obb.cpp src/physics/collision.cpp)
target_compile_options(DatasetGenerator PRIVATE -O2)
target_include_directories(DatasetGenerator PRIVATE ${CMAKE_SOURCE_DIR}/include)

# ---------------------------------------------------------------------------
# Headless scene validation harness (no OpenGL). Loads every registered Scene
# through the real SceneManager + solver and checks physical invariants
# (finite, no explosion, deck-stays-suspended, etc.). Reproduces scene crashes
# deterministically. Build with:  cmake --build build --target SceneTests
# ---------------------------------------------------------------------------
add_executable(SceneTests src/app/scene_tests.cpp src/physics/physicssolver.cpp src/physics/obb.cpp src/physics/collision.cpp)
target_compile_options(SceneTests PRIVATE -O2)
target_include_directories(SceneTests PRIVATE ${CMAKE_SOURCE_DIR}/include)

# ---------------------------------------------------------------------------
# Physics Validation Laboratory (no OpenGL). Runs analytic experiments through
# the four validation layers (A numerical integrity, B constraint validation,
# C physical-law analytical comparison with %% error, D randomized stress
# testing). Deterministic; the scientific regression benchmark for the engine.
# Build with:  cmake --build build --target ValidationSuite
# ---------------------------------------------------------------------------
add_executable(ValidationSuite src/app/validation_suite.cpp src/physics/physicssolver.cpp src/physics/obb.cpp src/physics/collision.cpp)
target_compile_options(ValidationSuite PRIVATE -O2)
target_include_directories(ValidationSuite PRIVATE ${CMAKE_SOURCE_DIR}/include)