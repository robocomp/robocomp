cmake_minimum_required( VERSION 2.6 )

collect_library_directory_names(${RoboComp_LIBS_DIR} "CMakeLists.txt" RoboComp_MODULES_NAMES RoboComp_MODULES_DIRS doc)

set(RoboComp_MODULES_NAMES_UNSORTED ${RoboComp_MODULES_NAMES})

topological_sort(RoboComp_MODULES_NAMES RoboComp_ _DEPENDS)

sort_relative(RoboComp_MODULES_NAMES_UNSORTED RoboComp_MODULES_NAMES RoboComp_MODULES_DIRS)

foreach(subdir ${RoboComp_MODULES_DIRS})
  add_subdirectory(${RoboComp_LIBS_DIR}/${subdir})  
endforeach(subdir)
