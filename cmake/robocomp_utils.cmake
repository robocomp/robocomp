###############################################################################
# Set the destination directories for installing stuff.
# Sets LIB_INSTALL_DIR. Install libraries here.
# Sets BIN_INSTALL_DIR. Install binaries here.
# Sets INCLUDE_INSTALL_DIR. Install include files here, preferably in a
# subdirectory named after the library in question (e.g.
# "registration/blorgle.h")


macro(SET_INSTALL_DIRS)
    set(INSTALL_DIR
        "/opt/${PROJECT_NAME_LOWER}")
        
    set(LIB_INSTALL_DIR "${INSTALL_DIR}/lib")
    set(INCLUDE_INSTALL_DIR "${INSTALL_DIR}/include")
    set(DOC_INSTALL_DIR "${INSTALL_DIR}/doc")
    set(BIN_INSTALL_DIR "${INSTALL_DIR}/bin")
    set(CONFIG_INSTALL_DIR "${INSTALL_DIR}/etc-default")
    set(PKGCFG_INSTALL_DIR "${LIB_INSTALL_DIR}/pkgconfig")
endmacro(SET_INSTALL_DIRS)

##########################################################################
# This function were copied from boost-cmake project.                    #
# The license terms is as follow                                         #
##########################################################################
# Copyright (C) 2007 Douglas Gregor <doug.gregor@gmail.com>              #
# Copyright (C) 2007 Troy Straszheim                                     #
#                                                                        #
# Distributed under the Boost Software License, Version 1.0.             #
# See accompanying file LICENSE_1_0.txt or copy at                       #
#   http://www.boost.org/LICENSE_1_0.txt                                 #
##########################################################################
# Perform a reverse topological sort on the given LIST. 
#   
#   topological_sort(my_list "MY_" "_EDGES")
#
# LIST is the name of a variable containing a list of elements to be
# sorted in reverse topological order. Each element in the list has a
# set of outgoing edges (for example, those other list elements that
# it depends on). In the resulting reverse topological ordering
# (written back into the variable named LIST), an element will come
# later in the list than any of the elements that can be reached by
# following its outgoing edges and the outgoing edges of any vertices
# they target, recursively. Thus, if the edges represent dependencies
# on build targets, for example, the reverse topological ordering is
# the order in which one would build those targets.
#
# For each element E in this list, the edges for E are contained in
# the variable named ${PREFIX}${E}${SUFFIX}, where E is the
# upper-cased version of the element in the list. If no such variable
# exists, then it is assumed that there are no edges. For example, if
# my_list contains a, b, and c, one could provide a dependency graph
# using the following variables:
#
#     MY_A_EDGES     b
#     MY_B_EDGES     
#     MY_C_EDGES     a b
#
#  With the involcation of topological_sort shown above and these
#  variables, the resulting reverse topological ordering will be b, a,
#  c.

macro(topological_sort LIST PREFIX SUFFIX)
    # Clear the stack and output variable
    set(VERTICES "${${LIST}}")
    set(STACK)
    set(${LIST})

    # Loop over all of the vertices, starting the topological sort from
    # each one.
    foreach(VERTEX ${VERTICES})
        string(TOUPPER ${VERTEX} UPPER_VERTEX)

        # If we haven't already processed this vertex, start a depth-first
        # search from where.
        if (NOT FOUND_${UPPER_VERTEX})
            # Push this vertex onto the stack with all of its outgoing edges
            string(REPLACE ";" " " NEW_ELEMENT 
                "${VERTEX};${${PREFIX}${UPPER_VERTEX}${SUFFIX}}")
            list(APPEND STACK ${NEW_ELEMENT})

            # We've now seen this vertex
            set(FOUND_${UPPER_VERTEX} TRUE)

            # While the depth-first search stack is not empty
            list(LENGTH STACK STACK_LENGTH)
            while(STACK_LENGTH GREATER 0)
                # Remove the vertex and its remaining out-edges from the top
                # of the stack
                list(GET STACK -1 OUT_EDGES)
                list(REMOVE_AT STACK -1)

                # Get the source vertex and the list of out-edges
                separate_arguments(OUT_EDGES)
                list(GET OUT_EDGES 0 SOURCE)
                list(REMOVE_AT OUT_EDGES 0)

                # While there are still out-edges remaining
                list(LENGTH OUT_EDGES OUT_DEGREE)
                while (OUT_DEGREE GREATER 0)
                    # Pull off the first outgoing edge
                    list(GET OUT_EDGES 0 TARGET)
                    list(REMOVE_AT OUT_EDGES 0)

                    string(TOUPPER ${TARGET} UPPER_TARGET)
                    if (NOT FOUND_${UPPER_TARGET})
                        # We have not seen the target before, so we will traverse
                        # its outgoing edges before coming back to our
                        # source. This is the key to the depth-first traversal.

                        # We've now seen this vertex
                        set(FOUND_${UPPER_TARGET} TRUE)

                        # Push the remaining edges for the current vertex onto the
                        # stack
                        string(REPLACE ";" " " NEW_ELEMENT 
                            "${SOURCE};${OUT_EDGES}")
                        list(APPEND STACK ${NEW_ELEMENT})

                        # Setup the new source and outgoing edges
                        set(SOURCE ${TARGET})
                        string(TOUPPER ${SOURCE} UPPER_SOURCE)
                        set(OUT_EDGES 
                            ${${PREFIX}${UPPER_SOURCE}${SUFFIX}})
                    endif(NOT FOUND_${UPPER_TARGET})

                    list(LENGTH OUT_EDGES OUT_DEGREE)
                endwhile (OUT_DEGREE GREATER 0)

                # We have finished all of the outgoing edges for
                # SOURCE; add it to the resulting list.
                list(APPEND ${LIST} ${SOURCE})

                # Check the length of the stack
                list(LENGTH STACK STACK_LENGTH)
            endwhile(STACK_LENGTH GREATER 0)
        endif (NOT FOUND_${UPPER_VERTEX})
    endforeach(VERTEX)
    # Somewhere a # slaps into the list so remove it
    list(REMOVE_ITEM ${LIST} "#")
endmacro(topological_sort)


###
# Sorts list B the same way list A was sorted by fetching the indices
# _list [IN] original list A 
# _sorted_list [IN] list A after sorting
# _to_sort_relative [IN/OUT] list B
##
macro(sort_relative _list _sorted_list _to_sort_relative)
  unset(sorted_list_length)
  unset(list_length)
  unset(to_sort_list_length)
  # ensure sizes are equal for the three lists else fail gracefully
  list(LENGTH ${_sorted_list} sorted_list_length)
  list(LENGTH ${_list} list_length)
  list(LENGTH ${_to_sort_relative} to_sort_list_length)

  if(NOT (list_length EQUAL sorted_list_length))
    message(STATUS "Original list: ${${_list}}")
    message(STATUS "Sorted list: ${${_sorted_list}}")
    message(FATAL_ERROR "size mismatch between ${_sorted_list} (length ${sorted_list_length}) and ${_list} (length ${list_length})")
  endif(NOT (list_length EQUAL sorted_list_length))

  if(NOT (list_length EQUAL to_sort_list_length))
    message(FATAL_ERROR "size mismatch between ${_to_sort_relative} ${to_sort_list_length} and ${_list} ${list_length}")
  endif(NOT (list_length EQUAL to_sort_list_length))
  # unset the temporary list to avoid suprises (I had some them and were hard to find)
  unset(tmp_list)
  # fill it with a dummy value
  fill_list(tmp_list list_length "#")
  #iterate over the original list
  set(counter 0)
  foreach(loop_var ${${_list}})
    # get the element position in the sorted list
    list(FIND ${_sorted_list} ${loop_var} sorted_position)
    # get the corresponding element from the list to sort
    list(GET ${_to_sort_relative} ${counter} to_insert)
    # in the temporary list replace the dummy value by the corresponding
    set_in_list(tmp_list sorted_position to_insert)
    # increment the counter
    math(EXPR counter "${counter} + 1")
  endforeach(loop_var)
  # swap the temporary list and list to sort
  set(${_to_sort_relative} ${tmp_list})
endmacro(sort_relative)

##
# Fills a list with _length x _value
# _list the list to fill
# _length the desired list size
# _value the filler
##
macro(fill_list _list _length _value)
  if(${_length} LESS 1)
    message(FATAL_ERROR "${_length} must be at least equal to 1")
  endif(${_length} LESS 1)
  math(EXPR size "${${_length}} - 1")
  foreach(counter RANGE ${size})
    list(APPEND ${_list} ${_value})
  endforeach(counter)
endmacro(fill_list)

##
# Set the value at element a known position of a list
# _list the list to manipulate
# _position position of the element to set
# _value new element value
##
macro(set_in_list _list _position _value)
  list(INSERT ${_list} ${${_position}} ${${_value}})
  math(EXPR next "${${_position}} + 1")
  list(REMOVE_AT ${_list} ${next})
endmacro(set_in_list)

###############################################################################
# Set a value in a global, cached map.
# _map The map name.
# _key The key name.
# _value The value.
macro(SET_IN_GLOBAL_MAP _map _key _value)
    set("${_map}_${_key}" "${_value}" CACHE INTERNAL "Map value" FORCE)
endmacro(SET_IN_GLOBAL_MAP)

###############################################################################
# Get a value from a map.
# _dest The name of the variable to store the value in.
# _map The map name.
# _key The key name.
macro(GET_IN_MAP _dest _map _key)
    set(${_dest} ${${_map}_${_key}})
endmacro(GET_IN_MAP)