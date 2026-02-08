project(CFE_RADIO_APP C)

# Create the app module
add_cfe_app(radio_murgas_app fsw/src/radio_app.c)

add_cfe_tables(radio_murgas_app fsw/tables/radio_app_tbl.c)

target_include_directories(radio_murgas_app PUBLIC
  fsw/mission_inc
  fsw/platform_inc
  ../bus_comms/fsw/src
  ../bus_comms/fsw/platform_inc
  ../cf/fsw/inc
  ../cf/config
)

# If UT is enabled, then add the tests from the subdirectory
if (ENABLE_UNIT_TESTS)
  add_subdirectory(unit-test)
endif (ENABLE_UNIT_TESTS)