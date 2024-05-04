cmake_minimum_required(VERSION 3.5)
project(prueba_1)

set(CMAKE_CXX_STANDARD 17)

find_package(pybind11 REQUIRED)

pybind11_add_module(prueba_1 prueba_1.cpp)

# Crear el módulo Python utilizando pybind11
target_link_libraries(prueba_1 PRIVATE pybind11::module)
