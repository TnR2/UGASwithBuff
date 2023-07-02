#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "UGAS" for configuration "Release"
set_property(TARGET UGAS APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(UGAS PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/UGAS"
  )

list(APPEND _cmake_import_check_targets UGAS )
list(APPEND _cmake_import_check_files_for_UGAS "${_IMPORT_PREFIX}/bin/UGAS" )

# Import target "Control" for configuration "Release"
set_property(TARGET Control APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Control PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libControl.a"
  )

list(APPEND _cmake_import_check_targets Control )
list(APPEND _cmake_import_check_files_for_Control "${_IMPORT_PREFIX}/lib/libControl.a" )

# Import target "Core" for configuration "Release"
set_property(TARGET Core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libCore.a"
  )

list(APPEND _cmake_import_check_targets Core )
list(APPEND _cmake_import_check_files_for_Core "${_IMPORT_PREFIX}/lib/libCore.a" )

# Import target "ThirdParty" for configuration "Release"
set_property(TARGET ThirdParty APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ThirdParty PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libThirdParty.a"
  )

list(APPEND _cmake_import_check_targets ThirdParty )
list(APPEND _cmake_import_check_files_for_ThirdParty "${_IMPORT_PREFIX}/lib/libThirdParty.a" )

# Import target "Util" for configuration "Release"
set_property(TARGET Util APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Util PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libUtil.a"
  )

list(APPEND _cmake_import_check_targets Util )
list(APPEND _cmake_import_check_files_for_Util "${_IMPORT_PREFIX}/lib/libUtil.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
