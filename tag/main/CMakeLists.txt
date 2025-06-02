
idf_component_register(SRCS "rpi_comm_task.cpp" "led_task.cpp" "gps_task.cpp" "gps_parser.cpp" "gps_tag_main.cpp" "nvs_util.c" "minmea.c" "nvs_util.c"
                       INCLUDE_DIRS ".")
#					   REQUIRES esp_timer driver nvs_flash)
#set(EXTRA_COMPONENT_DIRS
#	${CMAKE_CURRENT_SOURCE_DIR}/../../common/switch_driver
#	)
ADD_DEFINITIONS( -Dtimegm=mktime)
target_compile_options(${COMPONENT_LIB} PRIVATE -fexceptions)
