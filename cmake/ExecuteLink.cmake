# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later

# ---------------------------------------------------------------------------
# Create a function to create and link a executable with the
# libraries defined in this project.
function(create_link_executable EXECUTABLE_NAME LIBRARIES SOURCES)
  add_executable(
      ${PROJECT_NAME}_${EXECUTABLE_NAME}
      ${SOURCES})
  target_include_directories(${PROJECT_NAME}_${EXECUTABLE_NAME}
      PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/examples
        )
  if(UNIX AND NOT APPLE)
    # Use DT_RPATH so transitive deps (e.g., GTSAM -> libcephes-gtsam) resolve.
    target_link_options(${PROJECT_NAME}_${EXECUTABLE_NAME} PRIVATE -Wl,--disable-new-dtags)
  endif()
  target_link_libraries(${PROJECT_NAME}_${EXECUTABLE_NAME}
      PRIVATE ${${LIBRARIES}})
endfunction()
