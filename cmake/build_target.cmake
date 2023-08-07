function(BuildTarget path)
    file(GLOB homeworks ${path}/*)

    foreach(subHW ${homeworks})
        get_filename_component(target_name ${subHW} NAME)
        file(GLOB_RECURSE sources ${subHW}/*.cpp ${subHW}/*.h)
        set(target_name "${target_name}")

        # executable
        add_executable(${target_name} ${sources})
        
        # opencv eigen
        target_include_directories(${target_name} PRIVATE ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(${target_name} PRIVATE opencv_highgui)

        # install
        install(TARGETS ${target_name} RUNTIME DESTINATION .)

    endforeach(subHW ${homeworks})

endfunction(BuildTarget path)