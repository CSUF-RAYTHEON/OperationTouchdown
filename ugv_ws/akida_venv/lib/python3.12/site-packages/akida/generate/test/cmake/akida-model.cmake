# ============================================================================
# To use the Akida model API with generated fixtures you have to include the
# CMake folder and link the library akida to your CMake target as following:
#
# set(CMAKE_INCLUDE_CURRENT_DIR ON)
# set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
# include(akida-model)
# target_link_libraries(my_program PRIVATE akida)
# ============================================================================
cmake_minimum_required(VERSION 3.16)
# Find python, and where it stores libraries
set(Python_FIND_VIRTUALENV FIRST)
set(Python_FIND_REGISTRY LAST)
find_package(Python REQUIRED COMPONENTS Interpreter)
execute_process(
    COMMAND
        ${Python_EXECUTABLE} -c "import sys; [print(p, end=';') for p in sys.path if p.strip()]"
    OUTPUT_VARIABLE
        Python_SysPath
)

message(STATUS "Searching akida library in: ${Python_SysPath}")
# Now find akida library in these paths
find_library(AKIDA_LIB
                NAMES
                    akida libakida.so.2
                PATHS
                    ${Python_SysPath}
                PATH_SUFFIXES
                    akida
                REQUIRED
)
message(STATUS "Found akida library at '${AKIDA_LIB}'")

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if (WIN32)
    install(FILES ${AKIDA_LIB} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()

# Extra path of found akida library
get_filename_component(AKIDA_PATH ${AKIDA_LIB} DIRECTORY)

add_library(akida INTERFACE)
target_link_libraries(akida INTERFACE ${AKIDA_LIB})
target_include_directories(akida INTERFACE
    ${AKIDA_PATH}/api
    ${AKIDA_PATH}/engine/api/
    ${AKIDA_PATH})
