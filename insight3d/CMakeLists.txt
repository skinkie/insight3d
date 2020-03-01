cmake_minimum_required(VERSION 3.10)
set(CMAKE_OSX_DEPLOYMENT_TARGET "10.13" CACHE STRING "Minimum OS X deployment version")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DGTK_DISABLE_SINGLE_INCLUDES")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DGDK_DISABLE_DEPRECATED -DGTK_DISABLE_DEPRECATED")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DGSEAL_ENABLE")

find_package(OpenCV 2.4 REQUIRED PATHS /usr/local/opt/opencv@2/share/OpenCV)
find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Freetype REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

find_library(ANN_LIBRARY ann)
find_library(OPENSIFT_LIBRARY opensift/libopensift.a)
find_library(SBA_LIBRARY sba)

find_path(ANN_INCLUDE ANN/ANN.h)
find_path(OPENSIFT_INCLUDE opensift/sift.h)
find_path(SBA_INCLUDE sba.h)

message("ANN_LIBRARY ${ANN_LIBRARY}")
message("OPENSIFT_LIBRARY ${OPENSIFT_LIBRARY}")
message("SBA_LIBRARY ${SBA_LIBRARY}")
message("GTK3_LIBRARY_DIRS ${GTK3_LIBRARY_DIRS}")
message("GTK3_LIBRARIES ${GTK3_LIBRARIES}")

message("ANN_INCLUDE ${ANN_INCLUDE}")
message("OPENSIFT_INCLUDE ${OPENSIFT_INCLUDE}")
message("SBA_INCLUDE ${SBA_INCLUDE}")
message("GTK3_INCLUDE_DIRS ${GTK3_INCLUDE_DIRS}")

include_directories(
	${OpenCV_INCLUDE_DIRS}
	${OPENGL_INCLUDE_DIRS}
	${SDL2_INCLUDE_DIRS}
	${FREETYPE_INCLUDE_DIRS}
	${LIBXML2_INCLUDE_DIRS}
	${LAPACK_INCLUDE_DIRS}
	${BLAS_INCLUDE_DIRS}
	${GTK3_INCLUDE_DIRS}
	${ANN_INCLUDE}
	${OPENSIFT_INCLUDE}
	${SBA_INCLUDE}
)

link_directories(
	${GTK3_LIBRARY_DIRS}
)

add_definitions(
	${GTK3_CFLAGS_OTHER}
)

file(GLOB INSIGHT_SOURCES
    "*.cpp"
)

file(GLOB INSIGHT_HEADERS
    "*.h"
)

add_executable(Insight3D
    ${INSIGHT_SOURCES}
    ${INSIGHT_HEADERS}
)

source_group(
    "Headers" REGULAR_EXPRESSION ".*\\.h"
)

target_link_libraries(Insight3D
	${OpenCV_LIBRARIES}
	${OPENGL_LIBRARIES}
	${SDL2_LIBRARIES}
	${FREETYPE_LIBRARIES}
	${LIBXML2_LIBRARIES}
	${LAPACK_LIBRARIES}
	${BLAS_LIBRARIES}
	${GTK3_LIBRARIES}
	${ANN_LIBRARY}
	${OPENSIFT_LIBRARY}
	${SBA_LIBRARY}
)
