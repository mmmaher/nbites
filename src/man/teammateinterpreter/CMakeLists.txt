cmake_minimum_required(VERSION 2.8.3)

set(TEAMMATE_SRCS TeammateInterpreterModule.cpp
)

add_library(
  teammateinterpreter
  STATIC
  ${TEAMMATE_SRCS}
)

target_link_libraries(
  teammateinterpreter
  support
  portals
  messages
)
