cmake_minimum_required(VERSION 3.10)

# 如果用户没指定，则默认用 Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 给 Release 配置一些常用优化标志（可选）
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

project(RayTracing)

# 启用 C++17 标准并关闭编译器扩展
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找 OpenMP 库（必需）
find_package(OpenMP REQUIRED)

# 添加可执行文件
add_executable(RayTracing 
    main.cpp 
    Vector.cpp Vector.hpp 
    Scene.cpp Scene.hpp 
    BVH.cpp BVH.hpp Bounds3.hpp
    Renderer.cpp Renderer.hpp
    Object.hpp Sphere.hpp global.hpp Triangle.hpp 
    Light.hpp AreaLight.hpp Ray.hpp Material.hpp Intersection.hpp
)

# 链接 OpenMP 到目标
target_link_libraries(RayTracing PUBLIC OpenMP::OpenMP_CXX)

# 可选：输出 OpenMP 配置信息（调试用）
if(OpenMP_FOUND)
    message(STATUS "OpenMP found! Version: ${OpenMP_VERSION}")
    message(STATUS "Compiler flags: ${OpenMP_CXX_FLAGS}")
endif()
