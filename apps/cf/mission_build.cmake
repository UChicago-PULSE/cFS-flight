###########################################################
#
# CF App mission build setup
#
# This file is evaluated as part of the "prepare" stage
# and can be used to set up prerequisites for the build,
# such as generating header files
#
###########################################################

# Add stand alone documentation
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/docs/dox_src ${MISSION_BINARY_DIR}/docs/cf-usersguide)

# The list of header files that control the CF configuration
set(CF_MISSION_CONFIG_FILE_LIST
  cf_extern_typedefs.h
  cf_fcncode_values.h
  cf_interface_cfg_values.h
  cf_mission_cfg.h
  cf_msgdefs.h
  cf_msg.h
  cf_msgstruct.h
  cf_tbldefs.h
  cf_tbl.h
  cf_tblstruct.h
  cf_topicid_values.h
)

# Create wrappers around the all the config header files
# This makes them individually overridable by the missions, without modifying
# the distribution default copies
foreach(CF_CFGFILE ${CF_MISSION_CONFIG_FILE_LIST})
  get_filename_component(CFGKEY "${CF_CFGFILE}" NAME_WE)
  if (DEFINED CF_CFGFILE_SRC_${CFGKEY})
    set(DEFAULT_SOURCE GENERATED_FILE "${CF_CFGFILE_SRC_${CFGKEY}}")
  else()
    set(DEFAULT_SOURCE FALLBACK_FILE "${CMAKE_CURRENT_LIST_DIR}/config/default_${CF_CFGFILE}")
  endif()
  generate_config_includefile(
    FILE_NAME           "${CF_CFGFILE}"
    ${DEFAULT_SOURCE}
  )
endforeach()