#-------------------------------------------------------------------------------
# Downloading test data from Amazon AWS
#-------------------------------------------------------------------------------
function(DownloadData)

    message(STATUS "...Downloading data for demos and tests")

    set(AWS_URL https://frydom-ce-data.s3.amazonaws.com/demo)

    set(DATA_FILE_CE data_v${frydom_data_version}.tar.gz)

    set(OUT ${CMAKE_SOURCE_DIR}/data/ce/${DATA_FILE_CE})

    if (NOT EXISTS ${OUT})
        file(DOWNLOAD ${AWS_URL}/${DATA_FILE_CE} ${OUT} STATUS status)
        list(GET status 0 error_code)
        if( error_code )
            execute_process(
                    COMMAND
                    ${CMAKE_COMMAND} -E remove ${OUT}
                    WORKING_DIRECTORY
                    ${CMAKE_SOURCE_DIR}/data/ce
            )
        else()
            execute_process(
                    COMMAND
                    ${CMAKE_COMMAND} -E tar xzf ${OUT}
                    WORKING_DIRECTORY
                    ${CMAKE_SOURCE_DIR}/data/ce
            )
        endif()
    endif()
endfunction()

