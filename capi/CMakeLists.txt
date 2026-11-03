#
#   Copyright 2025 Pixar
#
#   Licensed under the terms set forth in the LICENSE.txt file available at
#   https://opensubdiv.org/license.
#

set(CAPI_HEADER subdiv_capi.h)
set(CAPI_SOURCE subdiv_capi.cpp)
set(CAPI_TEST_SOURCE test_capi.c)

include_directories(
    "${PROJECT_SOURCE_DIR}"
)

add_library(osd_capi SHARED ${CAPI_SOURCE} ${CAPI_HEADER})

target_compile_definitions(osd_capi
    PRIVATE
        OSD_CAPI_EXPORTS
)

set_target_properties(osd_capi
    PROPERTIES
        OUTPUT_NAME OpenSubdivCAPI
        FOLDER "opensubdiv/capi"
)

target_link_libraries(osd_capi
    osd_static_cpu
)

if (WIN32)
    set_target_properties(osd_capi PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS OFF)
endif()

install( TARGETS osd_capi
    RUNTIME DESTINATION "${CMAKE_BINDIR_BASE}"
    LIBRARY DESTINATION "${CMAKE_LIBDIR_BASE}"
    ARCHIVE DESTINATION "${CMAKE_LIBDIR_BASE}" )

install( FILES ${CAPI_HEADER}
    DESTINATION
        "${CMAKE_INCDIR_BASE}"
    PERMISSIONS
        OWNER_READ
        GROUP_READ
        WORLD_READ )

add_executable(capi_test ${CAPI_TEST_SOURCE})

set_target_properties(capi_test
    PROPERTIES
        FOLDER "opensubdiv/capi"
)

target_link_libraries(capi_test
    osd_capi
)

add_dependencies(capi_test osd_capi)

if (NOT NO_TESTS)
    add_test(NAME capi_test COMMAND capi_test)
endif()
