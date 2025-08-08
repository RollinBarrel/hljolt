cmake_minimum_required(VERSION 3.20)

project(jolt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(libs/JoltPhysics/Build/)

add_library(jolt.hdll SHARED jolt.cpp)
target_link_libraries(jolt.hdll Jolt)
# target_include_directories(jolt.hdll PUBLIC libs/jolt)
set_target_properties(jolt.hdll PROPERTIES PREFIX "" OUTPUT_NAME jolt SUFFIX .hdll)

IF(WIN32)
target_include_directories(jolt.hdll PUBLIC $ENV{HASHLINKPATH}/include)
target_link_libraries(jolt.hdll $ENV{HASHLINKPATH}/libhl.lib)
ELSEIF(APPLE)
find_library(APPLICATION_SERVICES_FRAMEWORK ApplicationServices REQUIRED)
target_include_directories(jolt.hdll PUBLIC $ENV{HASHLINKPATH}/src)
target_link_libraries(jolt.hdll $ENV{HASHLINKPATH}/libhl.dylib ${APPLICATION_SERVICES_FRAMEWORK})
ELSEIF()
target_include_directories(jolt.hdll PUBLIC $ENV{HASHLINKPATH}/src)
target_link_libraries(jolt.hdll $ENV{HASHLINKPATH}/libhl.so)
ENDIF()

install(TARGETS jolt.hdll DESTINATION lib)