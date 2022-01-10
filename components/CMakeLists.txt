
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/components)
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)

idf_component_register(SRCS 
"mime.c"
"sntp_task.c"
"sockets.c"

                    INCLUDE_DIRS ".")
                    
