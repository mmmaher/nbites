cmake_minimum_required( VERSION 2.8.3 )

set( LOGSHARE_SRCS Log SExpr )

add_library(
  logshare
  STATIC
  ${LOGSHARE_SRCS}
)
