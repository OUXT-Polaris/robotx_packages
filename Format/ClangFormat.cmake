## define format
set(CODING_FORMAT "{BasedOnStyle: Google, ColumnLimit: 110, BinPackParameters: false}")

## functions  
option(WITH_CLANG_FORMAT ON)
find_program(CLANG_FORMAT_EXE clang-format)

function(clang_format target)
  if(CLANG_FORMAT_EXE)
    message(STATUS "Enable Clang-Format ${target}")
    get_target_property(MY_SOURCES ${target} SOURCES)
    file(GLOB HEADERS include/*)

    add_custom_target(
      "${target}_format-with-clang-format"
      COMMAND "${CLANG_FORMAT_EXE}" -i -style=${CODING_FORMAT} ${MY_SOURCES} ${HEADERS}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      )
    if(WITH_CLANG_FORMAT)
      add_dependencies(${target} "${target}_format-with-clang-format")
    endif()
  endif()
endfunction()