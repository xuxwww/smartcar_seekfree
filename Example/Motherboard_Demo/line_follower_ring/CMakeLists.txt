cmake_minimum_required(VERSION 3.15)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

add_compile_options(-Ofast -flto)
add_link_options(-flto)

# ---- line_follower_ring ----
add_executable(${PROJECT_NAME} main.cpp image.c)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    SMTC2GO_DEBUG SMTC2GO_DEBUG_IMSHOW SMTC2GO_DEBUG_LOG
)
target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    zf::library
    template_schedule
    ${OpenCV_LIBS}
)
install(TARGETS ${PROJECT_NAME} DESTINATION .)

# ---- pid_debug ----
add_executable(pid_debug pid_debug.cpp)
target_link_libraries(pid_debug PRIVATE zf::library template_schedule pthread)
