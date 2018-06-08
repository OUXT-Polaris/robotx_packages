find_package(Doxygen)

function(add_document target)
    if(${DOXYGEN_FOUND})
        set(doxydir ${PROJECT_SOURCE_DIR}/../Doxygen)
        set(outputdir ${PROJECT_BINARY_DIR}/DoxyDoc)
        message("build document for ${target}")
        get_property(sourcefiles
            TARGET ${target}
            PROPERTY SOURCES)
        foreach (source ${sourcefiles})
            set(source_spaces "${source_spaces} ${source}")
        endforeach ()
        get_property(includedirs
            TARGET ${target}
            PROPERTY INCLUDE_DIRECTORIES)
        file(GLOB headers ${includedirs}/*.h)
        foreach (h ${headers})
            set(header_spaces "${header_spaces} ${h}")
        endforeach ()
        foreach (dir ${includedirs})
            set(dir_spaces "${dir_spaces} ${dir}")
        endforeach ()
        get_property(definitions
            DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            PROPERTY COMPILE_DEFINITIONS)
        foreach (def ${definitions})
            set(predef_spaces "${predef_spaces} ${def}")
        endforeach ()
        message("found source codes : ${source_spaces}")
        message("output directory : ${outputdir}")
        message("doxygen directory : ${doxydir}")
        file(MAKE_DIRECTORY ${outputdir})
        add_custom_command(
            OUTPUT  ${outputdir}/Doxyfile
            COMMAND ${CMAKE_COMMAND}
                    -D "DOXYGEN_TEMPLATE=${doxydir}/Doxyfile.in"
                    -D "DOXY_PROJECT_INPUT=${source_spaces} ${header_spaces}"
                    -D "DOXY_PROJECT_INCLUDE_DIR=${dir_spaces}"
                    -D "DOXY_PROJECT_PREDEFINED=${predef_spaces}"
                    -D "DOXY_PROJECT_STRIP_FROM_PATH=${PROJECT_SOURCE_DIR}"
                    -D "DOXY_DOCUMENTATION_OUTPUT_PATH=${outputdir}"
                    -D "DOXY_PROJECT_NAME=${target}"
                    -P "${doxydir}/doxygen-script.cmake"
            DEPENDS ${doxydir}/Doxyfile.in
                    ${outputdir}
            WORKING_DIRECTORY
                    ${outputdir}
            COMMENT "Generating Doxyfile for ${target}")
        add_custom_command(
            OUTPUT  ${outputdir}/${target}/index.html
            COMMAND ${DOXYGEN_EXECUTABLE}
            DEPENDS ${outputdir}/Doxyfile
            WORKING_DIRECTORY
                    ${outputdir}/${target}
            COMMENT "Creating HTML documentation for ${target}")
        add_custom_target(doxygen-${target}
            DEPENDS ${outputdir}/${target}/index.html)
        add_dependencies(doxygen
            doxygen-${targetname})
    else()
        message("doxygen not found")
    endif()
endfunction()