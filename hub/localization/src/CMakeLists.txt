include_directories(commonFiles
                    comunication
                    localization
                    ringBuffer
                    )
set(COMUNICATION_SRC
    comunication/Comunication.cpp
    comunication/Comunication.h)
set(LOCALIZATION_SRC
    localization/localization.cpp
    localization/localization.h)
set(COMMON_SRC
    commonFiles/common.h
    commonFiles/misc.h
    commonFiles/misc.cpp)


# Agregar los archivos fuente al ámbito superior
set(src_sources ${COMMUNICATION_SRC} ${LOCALIZATION_SRC} ${COMMON_SRC})
# Agregar ejecutable usando los archivos fuente de la carpeta src
add_executable(cli cli.cc)
# Enlaza con la biblioteca OpenCV
target_link_libraries(cli ${OpenCV_LIBRARIES})

# Opciones del compilador y bibliotecas adicionales
set(CLIBS -lpthread)
set(OPENCVLIBS -lopencv_highgui -lopencv_aruco -lopencv_imgcodecs -lopencv_core -lopencv_videoio -lopencv_calib3d -lopencv_imgproc)
set(ZMQLIBS -lzmq -lzmqpp)

# Agrega las opciones y bibliotecas al ejecutable
target_link_libraries(cli ${CLIBS} ${OPENCVLIBS} ${ZMQLIBS})