remake_find_library(
  epos epos.h
  PACKAGE libepos
)
remake_find_library(
  tulibs-config config.h
  PACKAGE tulibs
)
remake_find_library(
  tulibs-timer timer.h
  PACKAGE tulibs
)
remake_find_library(
  tulibs-thread thread.h
  PACKAGE tulibs
)
remake_find_library(
  tulibs-spline spline.h
  PACKAGE tulibs
)

remake_add_directories(lib)
remake_add_directories(bin)
