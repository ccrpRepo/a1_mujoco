cmake_minimum_required(VERSION 3.14)
project(quadsys)

set(CMAKE_CXX_STANDARD 11)

set(USE_GL 1)
set(BIN_NAME ${PROJECT_NAME})

# find_package(Eigen3 REQUIRED)

#Mujoco paths
set(MJPRO_PATH /home/crp/.mujoco/mujoco210)
include_directories(${MJPRO_PATH}/include)
link_directories(${MJPRO_PATH}/bin)
message(STATUS "MuJoCo path: " ${MJPRO_PATH})

#Finding main mujoco library
if(${USE_GL})
    file(GLOB LIB_MUJOCO ${MJPRO_PATH}/bin/libmujoco[0-9][0-9][0-9].so)
else()
    file(GLOB LIB_MUJOCO ${MJPRO_PATH}/bin/libmujoco[0-9][0-9][0-9]nogl.so)
endif()
#Showing mujoco library found
message(STATUS "MuJoCo lib: " ${LIB_MUJOCO})


include_directories(
    include
)

file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

add_executable(${BIN_NAME}
src/main.cpp
${SRC_LIST}
)

target_link_libraries(${BIN_NAME} ${LIB_MUJOCO} pthread)

#Standard libraries for GL
target_link_libraries(${PROJECT_NAME} GL GLU glut )

#Additional libraries from mujoco package
target_link_libraries(${PROJECT_NAME} ${MJPRO_PATH}/bin/libglew.so )
target_link_libraries(${PROJECT_NAME} ${MJPRO_PATH}/bin/libglfw.so.3 )
#target_link_libraries(${PROJECT_NAME}_exe libglewegl.so )
#target_link_libraries(${PROJECT_NAME}_exe libglewosmesa.so)

target_include_directories(${PROJECT_NAME} PUBLIC
 include
)