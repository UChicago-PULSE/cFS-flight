cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0079 NEW)
project(CFE_BUS_COMMS_APP C)

set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
set(ENABLE_CAN ON CACHE BOOL "" FORCE)
set(ENABLE_CAN_SOCKETCAN ON CACHE BOOL "" FORCE)
set(CSP_ENABLE_SOCKETCAN ON CACHE BOOL "" FORCE)

add_subdirectory(fsw/libcsp)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SOCKETCAN libsocketcan)

if(SOCKETCAN_FOUND)
  set(SOCKETCAN_LIB ${SOCKETCAN_LIBRARIES})
  set(SOCKETCAN_INCLUDE_DIRS ${SOCKETCAN_INCLUDE_DIRS})
else()
  find_library(SOCKETCAN_LIB
    NAMES socketcan
    PATHS
      /usr/lib/arm-linux-gnueabihf
      /usr/local/lib/arm-linux-gnueabihf
      /usr/arm-linux-gnueabihf/usr/lib/arm-linux-gnueabihf
      /usr/arm-linux-gnueabihf/usr/lib
      /usr/arm-linux-gnueabihf/lib
  )
  if(NOT SOCKETCAN_LIB)
    message(FATAL_ERROR "libsocketcan (armhf) not found")
  endif()
  set(SOCKETCAN_INCLUDE_DIRS
      /usr/include
      /usr/arm-linux-gnueabihf/usr/include)
endif()

target_include_directories(csp PUBLIC ${SOCKETCAN_INCLUDE_DIRS})
target_compile_definitions(csp PUBLIC
  CSP_HAVE_LIBSOCKETCAN=1
  ENABLE_CAN=1
  ENABLE_CAN_SOCKETCAN=1
  CSP_ENABLE_SOCKETCAN=1
)
target_link_libraries(csp PUBLIC ${SOCKETCAN_LIB})

add_cfe_app(bus_comms fsw/src/bus_comms_app.c)
add_cfe_tables(bus_comms fsw/tables/bus_comms_tbl.c)

target_include_directories(bus_comms PUBLIC
  fsw/mission_inc
  fsw/platform_inc
  fsw/libcsp/include
  ${SOCKETCAN_INCLUDE_DIRS}
)

find_package(Threads REQUIRED)

# Plain signature to match mission build usage elsewhere
target_link_libraries(bus_comms
  csp
  Threads::Threads
  rt
  ${SOCKETCAN_LIB}
)

if(TARGET driver_can)
  target_link_libraries(bus_comms driver_can)
endif()
if(TARGET if_can_socketcan)
  target_link_libraries(bus_comms if_can_socketcan)
endif()

set_target_properties(bus_comms PROPERTIES
  BUILD_RPATH "\$ORIGIN"
  INSTALL_RPATH "\$ORIGIN"
)

target_compile_options(bus_comms PRIVATE -Wno-error=pedantic)
target_compile_options(csp PRIVATE
  -Wno-error=pedantic
  -include ${CMAKE_CURRENT_SOURCE_DIR}/fsw/libcsp/libcsp_cfs_compat.h
)
if(TARGET driver_can)
  target_compile_options(driver_can PRIVATE
    -Wno-error=pedantic
    -include ${CMAKE_CURRENT_SOURCE_DIR}/fsw/libcsp/libcsp_cfs_compat.h
  )
endif()
if(TARGET if_zmq)
  target_compile_options(if_zmq PRIVATE
    -Wno-error=pedantic
    -include ${CMAKE_CURRENT_SOURCE_DIR}/fsw/libcsp/libcsp_cfs_compat.h
  )
endif()

# Copy libcsp.so next to the app for $ORIGIN rpath
add_custom_command(TARGET bus_comms POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:csp> $<TARGET_FILE_DIR:bus_comms>
)

# Install libcsp.so alongside bus_comms.so for each target CPU
if(APP_DYNAMIC_TARGET_LIST AND INSTALL_SUBDIR)
  foreach(TGT ${APP_DYNAMIC_TARGET_LIST})
    install(FILES $<TARGET_FILE:csp>
      DESTINATION ${TGT}/${INSTALL_SUBDIR}
    )
  endforeach()
endif()

# Copy optional driver .so only if it is a SHARED/MODULE lib or executable
function(_maybe_copy_target tgt)
  if(TARGET ${tgt})
    get_target_property(_t ${tgt} TYPE)
    if(_t STREQUAL "SHARED_LIBRARY" OR _t STREQUAL "MODULE_LIBRARY" OR _t STREQUAL "EXECUTABLE")
      add_custom_command(TARGET bus_comms POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${tgt}> $<TARGET_FILE_DIR:bus_comms>)
    endif()
  endif()
endfunction()

_maybe_copy_target(driver_can)
_maybe_copy_target(if_can_socketcan)

if (ENABLE_UNIT_TESTS)
endif()





