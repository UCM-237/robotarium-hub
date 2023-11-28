
CMAKE_MINIMUM_REQUIRED(VERSION 3.22)

# Establece el nombre del proyecto y su versión
project(Localization VERSION 1.0)

# Configura las opciones y estándares del compilador
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Configura las opciones de compilación (puedes añadir más según sea necesario)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Encuentra el paquete OpenCV
find_package(OpenCV REQUIRED)

# Agrega la carpeta de encabezados de OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# Agregar la carpeta src que contiene los archivos fuente
add_subdirectory(src)


