function(aegistrack_enable_sanitizers target_name)
    if(NOT AEGISTRACK_ENABLE_SANITIZERS)
        return()
    endif()

    if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(SANITIZER_FLAGS
            -fsanitize=address,undefined
            -fno-omit-frame-pointer
            -fno-sanitize-recover=all
        )
        target_compile_options(${target_name} INTERFACE ${SANITIZER_FLAGS})
        target_link_options(${target_name} INTERFACE ${SANITIZER_FLAGS})
    elseif(MSVC)
        target_compile_options(${target_name} INTERFACE /fsanitize=address /Zi)
    endif()
endfunction()
