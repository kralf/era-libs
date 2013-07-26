cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  era-libs
  VERSION 0.1
  RELEASE alpha
  SUMMARY "Driver libraries for the Embedded Robot Arm (ERA)."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/era-libs"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX era-
  INSTALL /usr
)

remake_doc(
  html
)

remake_pack_deb(
  DEPENDS libepos tulibs
)
remake_pack_deb(
  COMPONENT utils
  DESCRIPTION "utilities"
  DEPENDS era-libs
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  DEPENDS era-libs libepos-dev tulibs-dev
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libepos-dev tulibs-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE ERA_LIBS_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libepos-dev tulibs-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE ERA_LIBS_GIT_REVISION
)
