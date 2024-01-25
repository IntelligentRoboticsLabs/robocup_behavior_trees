cmake_minimum_required(VERSION 3.8)
project(robocup_behavior_trees)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(skills_library REQUIRED)
find_package(ament_index_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(dependencies
    skills_library
    ament_index_cpp
)
include_directories(include ${ZMQ_INCLUDE_DIRS})

add_executable(carry_my_luggage_main src/carry_my_luggage_main.cpp)
ament_target_dependencies(carry_my_luggage_main ${dependencies})
target_link_libraries(carry_my_luggage_main ${ZMQ_LIBRARIES})

add_executable(receptionist_main src/receptionist_main.cpp)
ament_target_dependencies(receptionist_main ${dependencies})
target_link_libraries(receptionist_main ${ZMQ_LIBRARIES})

add_executable(serve_breakfast_main src/serve_breakfast_main.cpp)
ament_target_dependencies(serve_breakfast_main ${dependencies})
target_link_libraries(serve_breakfast_main ${ZMQ_LIBRARIES})

add_executable(storing_groceries_main src/storing_groceries_main.cpp)
ament_target_dependencies(storing_groceries_main ${dependencies})
target_link_libraries(storing_groceries_main ${ZMQ_LIBRARIES})

install(TARGETS
  carry_my_luggage_main
  receptionist_main
  serve_breakfast_main
  storing_groceries_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY xml #launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
