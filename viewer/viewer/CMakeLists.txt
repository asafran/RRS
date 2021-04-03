file(GLOB SOURCES
    include/*.h
    src/*.cpp
)

find_package(Qt5 COMPONENTS OpenGL REQUIRED)

add_executable(viewer ${SOURCES})

#target_link_libraries(viewer ${OPENSCENEGRAPH_LIBRARIES})

target_link_libraries(viewer vsg::vsg)

target_link_libraries(viewer vsgXchange::vsgXchange)

#target_link_libraries(viewer ${OSG_QT_LIB})

target_link_libraries(viewer Qt::Widgets)

target_link_libraries(viewer Qt::Core)

target_link_libraries(viewer Qt5::OpenGL)

target_link_libraries(viewer CfgReader)

target_include_directories(viewer PRIVATE include)

target_include_directories(viewer PRIVATE /usr/local/include/osgQOpenGL)

target_include_directories(viewer PRIVATE ../../common-headers)

target_link_libraries(viewer filesystem)

target_link_libraries(viewer route-loader)

target_link_libraries(viewer display)

target_link_libraries(viewer library)

target_link_libraries(viewer display)

target_link_libraries(viewer display-player)

target_link_libraries(viewer TcpConnection)

target_link_libraries(viewer CfgReader)
target_link_libraries(viewer filesystem)
target_link_libraries(viewer Journal)
target_link_libraries(viewer physics)
target_link_libraries(viewer vehicle)
target_link_libraries(viewer coupling)
target_link_libraries(viewer solver)
target_link_libraries(viewer train)
target_link_libraries(viewer brakepipe)
target_link_libraries(viewer TcpConnection)
target_link_libraries(viewer sim-client)
target_link_libraries(viewer profile)
target_link_libraries(viewer device)
target_link_libraries(viewer model)

set_target_properties( viewer
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}../../bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}../../bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}../../bin"
)
