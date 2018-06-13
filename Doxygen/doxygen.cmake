find_package(Doxygen)

option(WITH_DOCUMENT OFF)

function(add_document target)
    if(${DOXYGEN_FOUND})
        set(doxydir ${PROJECT_SOURCE_DIR}/../Doxygen)
        set(outputdir ${PROJECT_BINARY_DIR}/DoxyDoc)
        set(image_source_dir ${PROJECT_SOURCE_DIR}/../Doxygen/images)
        set(image_dir ${PROJECT_BINARY_DIR}/DoxyDoc//${target}/html/images)
        #message("build document for ${target}")

        file(GLOB images_png ${image_source_dir}/*.png)
        file(GLOB images_jpg ${image_source_dir}/*.jpg)
        file(GLOB images_jpeg ${image_source_dir}/*.jpeg)
        #message("images : ${images_png} ${images_jpg} ${images_jpeg}")
        foreach (img ${images_png})
            file(COPY ${img} DESTINATION ${image_dir})
            set(image_spaces "${image_spaces} ${img}")
            #message("transport images : ${img}")
        endforeach ()
        foreach (img ${images_jpg})
            file(COPY ${img} DESTINATION ${image_dir})
            set(image_spaces "${image_spaces} ${img}")
            #message("transport images : ${img}")
        endforeach ()
        foreach (img ${images_jpeg})
            file(COPY ${img} DESTINATION ${image_dir})
            set(image_spaces "${image_spaces} ${img}")
            #message("transport images : ${img}")
        endforeach ()

        get_property(sourcefiles
            TARGET ${target}
            PROPERTY SOURCES)
        foreach (source ${sourcefiles})
            set(source_spaces "${source_spaces} ${PROJECT_SOURCE_DIR}/${source}")
        endforeach ()
        get_property(includedirs
            TARGET ${target}
            PROPERTY INCLUDE_DIRECTORIES)
        file(GLOB headers ${includedirs}/*.h)
        foreach (h ${headers})
            set(header_spaces "${header_spaces} ${h}")
        endforeach ()
        file(GLOB headers_hh ${includedirs}/*.hh)
        foreach (h ${headers_hh})
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
        #message("found source codes : ${source_spaces}")
        #message("output directory : ${outputdir}")
        #message("doxygen directory : ${doxydir}")
        #message("include directory : ${includedirs}")
        file(MAKE_DIRECTORY ${outputdir})
        add_custom_command(
            OUTPUT  ${outputdir}/${target}/Doxyfile
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
            DEPENDS ${outputdir}/${target}/Doxyfile
            WORKING_DIRECTORY
                    ${outputdir}/${target}
            COMMENT "Creating HTML documentation for ${target}")
        if(WITH_DOCUMENT)
            add_custom_target("doxygen-${PROJECT_NAME}-${target}" ALL
                DEPENDS ${outputdir}/${target}/index.html)
        else()
            add_custom_target("doxygen-${PROJECT_NAME}-${target}"
            DEPENDS ${outputdir}/${target}/index.html)
        endif()
    else()
        #message("doxygen not found")
    endif()
endfunction()