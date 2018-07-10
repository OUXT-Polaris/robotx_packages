## functions  
option(WITH_CLANG_FORMAT OFF)
find_program(CLANG_FORMAT_EXE clang-format)

function(clang_format target)
  if(CLANG_FORMAT_EXE)
    message(STATUS "Enable Clang-Format ${target}")
    get_target_property(MY_SOURCES ${target} SOURCES)
    file(GLOB HEADERS include/*)

    file(COPY ../Format/.clang-format DESTINATION ${PROJECT_SOURCE_DIR})

    add_custom_target(
      "${target}_format-with-clang-format"
      COMMAND "${CLANG_FORMAT_EXE}" -i -style=file ${MY_SOURCES} ${HEADERS}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      )
    if(WITH_CLANG_FORMAT)
      add_dependencies(${target} "${target}_format-with-clang-format")
    endif()
  endif()
endfunction()