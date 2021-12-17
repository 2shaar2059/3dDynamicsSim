cmake_minimum_required(VERSION 3.12)
project(3dDynamicsSim LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin/")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3")

include_directories("include/")

find_package(Eigen3 REQUIRED)
find_package(matplotlib_cpp REQUIRED)


#library creation
set(LIBRARY_NAME ${PROJECT_NAME})
include(GNUInstallDirs)
set(SOURCES src/RigidBody.cpp src/utils.cpp) # Find source files
add_library(${PROJECT_NAME} SHARED ${SOURCES}) # Create shared library
target_link_libraries(${PROJECT_NAME} ${matplotlib_cpp_LIBRARIES})
install(TARGETS ${PROJECT_NAME} EXPORT ${LIBRARY_NAME}Config
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

#adding the library's header files to the place CMAKE usually looks for header files
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

#adding the library's config file to the place CMAKE usually looks for config files (share)
install(EXPORT ${LIBRARY_NAME}Config DESTINATION share/cmake/${LIBRARY_NAME})




#building the executable targets of this library
find_package(3dDynamicsSim REQUIRED)

add_executable(3dDynamicsSim_main src/main.cpp)
target_link_libraries(3dDynamicsSim_main 3dDynamicsSim)

add_executable(test_matplotlib src/test_matplotlib.cpp)
target_link_libraries(test_matplotlib 3dDynamicsSim)