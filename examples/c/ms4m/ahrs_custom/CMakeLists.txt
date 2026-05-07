cmake_minimum_required(VERSION 3.16)

project("7_series_ahrs_custom_example_c"
    VERSION "1.0.0"
    DESCRIPTION "Simple example for the 3DM-CV7-AHRS, and 3DM-GV7-AHRS using C"
    LANGUAGES C CXX          # <-- Se agrega CXX para compilar main.cpp
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)              # main.cpp usa std::string, iostream, etc.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Resolución del MIP SDK root
# ---------------------------------------------------------------------------
if(DEFINED MIP_SDK_ROOT_DIR OR DEFINED CACHE{MIP_SDK_ROOT_DIR})
    set(_MIP_SDK_ROOT_DIR "${MIP_SDK_ROOT_DIR}")
elseif(DEFINED ENV{MIP_SDK_ROOT_DIR})
    set(_MIP_SDK_ROOT_DIR "$ENV{MIP_SDK_ROOT_DIR}")
else()
    set(_MIP_SDK_ROOT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
endif()

# ---------------------------------------------------------------------------
# Búsqueda de paquetes MIP / MicroStrain (solo necesarios para el INS)
# ---------------------------------------------------------------------------
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_LIST_DIR})
    list(APPEND CMAKE_PREFIX_PATH "${_MIP_SDK_ROOT_DIR}")

    find_package(mip REQUIRED CONFIG)

    # This example uses a serial connection and requires the microstrain_serial component
    find_package(microstrain REQUIRED COMPONENTS microstrain_serial CONFIG)
elseif(NOT MICROSTRAIN_ENABLE_SERIAL)
    message(FATAL_ERROR "The serial connections library is required for this example")
endif()

if(NOT DEFINED CACHE{MIP_LIBRARIES} AND NOT DEFINED MIP_LIBRARIES)
    message(FATAL_ERROR "Could not find the required MIP libraries.")
endif()

if(NOT DEFINED CACHE{MICROSTRAIN_LIBRARIES} AND NOT DEFINED MICROSTRAIN_LIBRARIES)
    message(FATAL_ERROR "Could not find the required MicroStrain libraries.")
endif()

# ---------------------------------------------------------------------------
# Opciones de compilación comunes
# ---------------------------------------------------------------------------
if(MSVC)
    set(COMPILE_OPTIONS
        "/W4"   # Enable warnings
        "/WX"   # Warnings as errors
        "/MP"   # Multi-process compilation
    )
else()
    set(COMPILE_OPTIONS
        "-Wall"
        "-Wextra"
        "-Werror"
    )
endif()

# ---------------------------------------------------------------------------
# TARGET 1: 7_series_ahrs_custom_example  (INS - C)
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}
    "${CMAKE_CURRENT_LIST_DIR}/7_series_ahrs_custom_example.c"
)

if(NOT ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_LIST_DIR})
    file(RELATIVE_PATH PROJECT_RELATIVE_PATH "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_LIST_DIR}/..")
    set_target_properties(${PROJECT_NAME} PROPERTIES
        FOLDER "${PROJECT_RELATIVE_PATH}"
    )
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${MIP_LIBRARIES}
    ${MICROSTRAIN_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${MIP_INCLUDE_DIRS}
    ${MICROSTRAIN_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME} PRIVATE ${COMPILE_OPTIONS})

target_compile_definitions(${PROJECT_NAME} PRIVATE
    "MICROSTRAIN_LOGGING_MAX_LEVEL=MICROSTRAIN_LOGGING_LEVEL_INFO_"
)

# ---------------------------------------------------------------------------
# TARGET 2: recording_tramaGPS  (GPS - C puro, sin dependencias externas)
# ---------------------------------------------------------------------------
add_executable(recording_tramaGPS
    "${CMAKE_CURRENT_LIST_DIR}/recording_tramaGPS.c"
)

target_compile_options(recording_tramaGPS PRIVATE ${COMPILE_OPTIONS})

# ---------------------------------------------------------------------------
# TARGET 3: launcher  (Orquestador - C++)
# Lanza los dos procesos anteriores via fork/exec y les envía teclas por pipe.
# IMPORTANTE: los binarios del INS y GPS deben estar en el mismo directorio
#             de trabajo que este ejecutable (o ajustar el PATH en main.cpp).
# ---------------------------------------------------------------------------
add_executable(launcher
    "${CMAKE_CURRENT_LIST_DIR}/main.cpp"
)   

target_compile_options(launcher PRIVATE ${COMPILE_OPTIONS})
target_link_libraries(launcher PRIVATE wiringPi)

# El launcher depende de que los otros dos targets estén listos primero
add_dependencies(launcher ${PROJECT_NAME} recording_tramaGPS)
