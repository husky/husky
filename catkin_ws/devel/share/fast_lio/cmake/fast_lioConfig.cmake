# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(fast_lio_CONFIG_INCLUDED)
  return()
endif()
set(fast_lio_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(fast_lio_SOURCE_PREFIX /home/kaijun/Documents/husky_autonomous/catkin_ws/src/FAST_LIO)
  set(fast_lio_DEVEL_PREFIX /home/kaijun/Documents/husky_autonomous/catkin_ws/devel)
  set(fast_lio_INSTALL_PREFIX "")
  set(fast_lio_PREFIX ${fast_lio_DEVEL_PREFIX})
else()
  set(fast_lio_SOURCE_PREFIX "")
  set(fast_lio_DEVEL_PREFIX "")
  set(fast_lio_INSTALL_PREFIX /home/kaijun/Documents/husky_autonomous/catkin_ws/install)
  set(fast_lio_PREFIX ${fast_lio_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'fast_lio' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(fast_lio_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/kaijun/Documents/husky_autonomous/catkin_ws/devel/include;/usr/include/eigen3;/usr/include/pcl-1.10;/usr/include;/usr/include/vtk-7.1;/usr/include/freetype2;/usr/include/x86_64-linux-gnu;/usr/include/ni;/usr/include/openni2 " STREQUAL " ")
  set(fast_lio_INCLUDE_DIRS "")
  set(_include_dirs "/home/kaijun/Documents/husky_autonomous/catkin_ws/devel/include;/usr/include/eigen3;/usr/include/pcl-1.10;/usr/include;/usr/include/vtk-7.1;/usr/include/freetype2;/usr/include/x86_64-linux-gnu;/usr/include/ni;/usr/include/openni2")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'claydergc <dev@livoxtech.com>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${fast_lio_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'fast_lio' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'fast_lio' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/kaijun/Documents/husky_autonomous/catkin_ws/src/FAST_LIO/${idir}'.  ${_report}")
    endif()
    _list_append_unique(fast_lio_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "/usr/lib/x86_64-linux-gnu/libpcl_common.so;/usr/lib/x86_64-linux-gnu/libpcl_kdtree.so;/usr/lib/x86_64-linux-gnu/libpcl_octree.so;/usr/lib/x86_64-linux-gnu/libpcl_search.so;/usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so;/usr/lib/x86_64-linux-gnu/libpcl_filters.so;/usr/lib/x86_64-linux-gnu/libpcl_io.so;/usr/lib/x86_64-linux-gnu/libpcl_features.so;/usr/lib/x86_64-linux-gnu/libpcl_ml.so;/usr/lib/x86_64-linux-gnu/libpcl_segmentation.so;/usr/lib/x86_64-linux-gnu/libpcl_visualization.so;/usr/lib/x86_64-linux-gnu/libpcl_surface.so;/usr/lib/x86_64-linux-gnu/libpcl_registration.so;/usr/lib/x86_64-linux-gnu/libpcl_keypoints.so;/usr/lib/x86_64-linux-gnu/libpcl_tracking.so;/usr/lib/x86_64-linux-gnu/libpcl_recognition.so;/usr/lib/x86_64-linux-gnu/libpcl_stereo.so;/usr/lib/x86_64-linux-gnu/libpcl_apps.so;/usr/lib/x86_64-linux-gnu/libpcl_outofcore.so;/usr/lib/x86_64-linux-gnu/libpcl_people.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so;/usr/lib/libOpenNI.so;/usr/lib/libOpenNI2.so;/usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libfreetype.so;/usr/lib/x86_64-linux-gnu/libz.so;/usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libjpeg.so;/usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libtiff.so;/usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libexpat.so;/usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libflann_cpp.so")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND fast_lio_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND fast_lio_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT fast_lio_NUM_DUMMY_TARGETS)
      set(fast_lio_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::fast_lio::wrapped-linker-option${fast_lio_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR fast_lio_NUM_DUMMY_TARGETS "${fast_lio_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::fast_lio::wrapped-linker-option${fast_lio_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND fast_lio_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND fast_lio_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND fast_lio_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/kaijun/Documents/husky_autonomous/catkin_ws/devel/lib;/opt/ros/noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(fast_lio_LIBRARY_DIRS ${lib_path})
      list(APPEND fast_lio_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'fast_lio'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND fast_lio_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(fast_lio_EXPORTED_TARGETS "fast_lio_generate_messages_cpp;fast_lio_generate_messages_eus;fast_lio_generate_messages_lisp;fast_lio_generate_messages_nodejs;fast_lio_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${fast_lio_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "geometry_msgs;nav_msgs;roscpp;rospy;std_msgs;message_runtime")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 fast_lio_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${fast_lio_dep}_FOUND)
      find_package(${fast_lio_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${fast_lio_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(fast_lio_INCLUDE_DIRS ${${fast_lio_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(fast_lio_LIBRARIES ${fast_lio_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${fast_lio_dep}_LIBRARIES})
  _list_append_deduplicate(fast_lio_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(fast_lio_LIBRARIES ${fast_lio_LIBRARIES})

  _list_append_unique(fast_lio_LIBRARY_DIRS ${${fast_lio_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(fast_lio_EXPORTED_TARGETS ${${fast_lio_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "fast_lio-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${fast_lio_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
