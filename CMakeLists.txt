cmake_minimum_required(VERSION 3.12)
project(3dDynamicsSim LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "build/")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3")

include_directories("include/" "/usr/include/python3.8")

find_package(Eigen3 REQUIRED)

#main executable creation
add_executable(main src/main.cpp src/utils.cpp src/RigidBody.cpp)
target_link_libraries(main Eigen3::Eigen python3.8)



#library creation
set(LIBRARY_NAME ${PROJECT_NAME})

include(GNUInstallDirs)

#set(SOURCES src/RigidBody.cpp src/utils.cpp) # Find source files
set(SOURCES src/RigidBody.cpp src/utils.cpp) # Find source files

add_library(${PROJECT_NAME} SHARED ${SOURCES}) # Create shared library

#target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME} EXPORT ${LIBRARY_NAME}Config
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

#adding the library's header files to the place CMAKE usually looks for header files
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
#install(FILES "../../matplotlib-cpp-master/matplotlibcpp.h" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/include)

#adding the library's config file to the place CMAKE usually looks for config files (share)
install(EXPORT ${LIBRARY_NAME}Config DESTINATION share/cmake/${LIBRARY_NAME})