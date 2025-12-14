find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)

set(SRC
    trajectory_loop_functions.cpp
    trajectory_loop_functions.hpp
    trajectory_qtuser_functions.cpp
    trajectory_qtuser_functions.hpp
    # Add other loop function source files here
)

add_library(loop_functions MODULE ${SRC})

target_link_libraries(loop_functions
  PUBLIC
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_pipuck
    Qt5::Widgets
    Qt5::OpenGL
)