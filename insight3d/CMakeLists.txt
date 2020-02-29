cmake_minimum_required(VERSION 3.10)
set(CMAKE_OSX_DEPLOYMENT_TARGET "10.13" CACHE STRING "Minimum OS X deployment version")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DGTK3_DISABLE_SINGLE_INCLUDES")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DGDK_DISABLE_DEPRECATED -DGTK3_DISABLE_DEPRECATED")
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

add_executable(Insight3D
	actions.cpp
	application.cpp
	core_constants.cpp
	core_debug.cpp
	core_image_loader.cpp
	core_math_routines.cpp
	core_state.cpp
	core_structures.cpp
	cv_extensions.cpp
	geometry_export.cpp
	geometry_loader.cpp
	geometry_publish.cpp
	geometry_queries.cpp
	geometry_routines.cpp
	geometry_structures.cpp
	geometry_textures.cpp
	gui.cpp
	gui_style.cpp
	interface_filesystem.cpp
	interface_opencv.cpp
	interface_opengl.cpp
	interface_sdl.cpp
	main.cpp
	mvg_autocalibration.cpp
	mvg_camera.cpp
	mvg_decomposition.cpp
	mvg_matching.cpp
	mvg_normalization.cpp
	mvg_resection.cpp
	mvg_triangulation.cpp
	tool_calibration.cpp
	tool_coordinates.cpp
	tool_core.cpp
	tool_edit.cpp
	tool_extrude.cpp
	tool_file.cpp
	tool_image.cpp
	tool_matching.cpp
	tool_plane_extraction.cpp
	tool_points.cpp
	tool_polygons.cpp
	tool_resection.cpp
	tool_selection.cpp
	tool_triangulation.cpp
	ui_constants.cpp
	ui_context.cpp
	ui_core.cpp
	ui_epipolars.cpp
	ui_events.cpp
	ui_inspection_mode.cpp
	ui_list.cpp
	ui_selection.cpp
	ui_shot_mode.cpp
	ui_state.cpp
	ui_visualization.cpp
	ui_visualization_helpers.cpp
	ui_visualization_point.cpp
	ui_workflow.cpp
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
