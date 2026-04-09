cmake_minimum_required(VERSION 4.1.2)
project(fatigue)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

set(SOURCES
    src/main.cpp
    src/MainWindow.cpp
    src/MainWindow.h
    src/SerialWorker.cpp
    src/SerialWorker.h
    src/Sample.cpp
    src/Sample.h
    src/SampleSink.h
)

find_package(nlohmann_json CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets SerialPort Charts)

qt_add_executable(fatigue ${SOURCES})

target_link_libraries(fatigue PRIVATE
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::SerialPort Qt6::Charts
)

if(WIN32)
    add_custom_command(TARGET fatigue POST_BUILD
        COMMAND "${Qt6_DIR}/../../../bin/windeployqt.exe"
                --release
                "$<TARGET_FILE:fatigue>"
        COMMENT "Running windeployqt..."
    )
endif()
