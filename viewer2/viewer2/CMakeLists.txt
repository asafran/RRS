file(GLOB SOURCES
    include/*.h
    src/*.cpp
)

find_package(Qt5 COMPONENTS OpenGL REQUIRED)

add_executable(viewer2 ${SOURCES})

#target_link_libraries(viewer2 ${OPENSCENEGRAPH_LIBRARIES})

target_link_libraries(viewer2 vsg::vsg)

target_link_libraries(viewer2 vsgXchange::vsgXchange)

#target_link_libraries(viewer2 ${OSG_QT_LIB})

target_link_libraries(viewer2 Qt::Widgets)

target_link_libraries(viewer2 Qt::Core)

target_link_libraries(viewer2 Qt5::OpenGL)

target_link_libraries(viewer2 CfgReader)

target_include_directories(viewer2 PRIVATE include)

target_include_directories(viewer2 PRIVATE /usr/local/include/osgQOpenGL)

target_include_directories(viewer2 PRIVATE ../../common-headers)

target_link_libraries(viewer2 filesystem)

target_link_libraries(viewer2 route-loader)

target_link_libraries(viewer2 display)

target_link_libraries(viewer2 library)

target_link_libraries(viewer2 display)

target_link_libraries(viewer2 display-player)

target_link_libraries(viewer2 TcpConnection)

target_link_libraries(viewer2 CfgReader)
target_link_libraries(viewer2 filesystem)
target_link_libraries(viewer2 Journal)
target_link_libraries(viewer2 physics)
target_link_libraries(viewer2 vehicle)
target_link_libraries(viewer2 coupling)
target_link_libraries(viewer2 solver)
target_link_libraries(viewer2 train)
target_link_libraries(viewer2 brakepipe)
target_link_libraries(viewer2 TcpConnection)
target_link_libraries(viewer2 sim-client)
target_link_libraries(viewer2 profile)
target_link_libraries(viewer2 device)
target_link_libraries(viewer2 model)

set_target_properties( viewer2
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}../../bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}../../bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}../../bin"
)
