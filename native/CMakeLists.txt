cmake_minimum_required(VERSION 3.7 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(flutter_bullet_library VERSION 1.0.0 LANGUAGES C CXX)
FILE (STRINGS "bullet3/VERSION" BULLET_VERSION)
include_directories(bullet3/src)
add_subdirectory(bullet3/src/LinearMath EXCLUDE_FROM_ALL)
add_subdirectory(bullet3/src/Bullet3Common EXCLUDE_FROM_ALL)
add_subdirectory(bullet3/src/BulletInverseDynamics EXCLUDE_FROM_ALL)
add_subdirectory(bullet3/src/BulletCollision EXCLUDE_FROM_ALL)
add_subdirectory(bullet3/src/BulletDynamics EXCLUDE_FROM_ALL)
add_subdirectory(bullet3/src/BulletSoftBody EXCLUDE_FROM_ALL)


add_library(flutter_bullet_library SHARED flutter_bullet.cc dart_api_dl.c)
target_link_libraries(flutter_bullet_library PUBLIC m)
target_link_libraries(flutter_bullet_library PUBLIC LinearMath)
target_link_libraries(flutter_bullet_library PUBLIC Bullet3Common)
target_link_libraries(flutter_bullet_library PUBLIC BulletInverseDynamics)
target_link_libraries(flutter_bullet_library PUBLIC BulletDynamics)
target_link_libraries(flutter_bullet_library PUBLIC BulletSoftBody)
target_link_libraries(flutter_bullet_library PUBLIC BulletCollision)

set_target_properties(flutter_bullet_library PROPERTIES
    PUBLIC_HEADER flutter_bullet.h
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    OUTPUT_NAME "flutter_bullet"
)

install(TARGETS flutter_bullet_library)
