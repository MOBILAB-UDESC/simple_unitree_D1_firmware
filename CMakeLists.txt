cmake_minimum_required(VERSION 3.12)

project(arm_mirror)

SET(CMAKE_CXX_STANDARD 17)

find_package(CycloneDDS-CXX REQUIRED)

function(generate_idlcxx_msgs TARGET_NAME FILE_PATH)
    idlcxx_generate(
        TARGET ${TARGET_NAME}
        FILES ${FILE_PATH}
        WARNINGS no-implicit-extensibility
    )
endfunction()

generate_idlcxx_msgs(servoangledatamsg src/msg/ServoAngleData.idl)
generate_idlcxx_msgs(servodatamsg src/msg/ServoData.idl)
generate_idlcxx_msgs(servopowermsg src/msg/ServoPower.idl)

add_executable(arm_slave src/arm_slave.cpp)

target_link_libraries(arm_slave
    fsuartservo
    cserialport
    CycloneDDS-CXX::ddscxx
    servoangledatamsg
    unitree_sdk2
    pthread
)

add_executable(arm_service src/arm_service.cpp)

target_link_libraries(arm_service
    fsuartservo
    cserialport
    CycloneDDS-CXX::ddscxx
    servodatamsg
    servoangledatamsg
    servopowermsg
    unitree_sdk2
    pthread
)