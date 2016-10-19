#pkg_version=1.4.1
cmake_minimum_required(VERSION 2.8.3)
project(imu_3dm_gx3_ros_tool)

###################
## find packages ##
###################
find_package(mrt_cmake_modules REQUIRED)
include(UseMrtStdCompilerFlags)
include(UseMrtAutoTarget)
include(MrtParamGenerator)

include(GatherDeps)
#remove libs, which cannot be found automatically
#list(REMOVE_ITEM DEPENDEND_PACKAGES <package name 1> <package name 2> ...)
find_package(AutoDeps REQUIRED COMPONENTS ${DEPENDEND_PACKAGES})

#manually resolve removed dependend packages
#find_package(...)
################################################
## Declare ROS messages, services and actions ##
################################################
# Add message, service and action files
glob_ros_files(add_message_files msg)
glob_ros_files(add_service_files srv)
glob_ros_files(add_action_files action)
# Generate added messages and services with any dependencies listed here
if (ROS_GENERATE_MESSAGES)
	generate_messages(
	  DEPENDENCIES
	  #add dependencies here
	  #std_msgs
	)
endif()
# Generate dynamic reconfigure options
file(GLOB PARAM_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "cfg/*.params")
file(GLOB CFG_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "cfg/*.cfg")
file(GLOB SRV_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "srv/*.srv")
if (PARAM_FILES OR CFG_FILES)
  generate_ros_parameter_files(
    ${PARAM_FILES}
    ${CFG_FILES}
  )
endif ()

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########
# Add include and library directories
include_directories(
  ${mrt_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${mrt_LIBRARY_DIRS}
)

function(add_exec EXEC_NAME SEARCH_FOLDER)
	#glob all files in this directory
	file(GLOB EXEC_SOURCE_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${SEARCH_FOLDER}/*.h" "${SEARCH_FOLDER}/*.hpp")
	file(GLOB EXEC_SOURCE_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${SEARCH_FOLDER}/*.cpp")
	if (EXEC_SOURCE_FILES_SRC)
		#add executable
		add_executable(${EXEC_NAME}
			${EXEC_SOURCE_FILES_INC}
			${EXEC_SOURCE_FILES_SRC}
		)
		# Add cmake target dependencies of the executable/library
		# as an example, message headers may need to be generated before nodes
		add_dependencies(${EXEC_NAME} ${catkin_EXPORTED_TARGETS})
		if (PARAM_FILES)
			add_dependencies(${EXEC_NAME} ${PROJECT_NAME}_genparam)
		endif()
		if (CFG_FILES)
			add_dependencies(${EXEC_NAME} ${PROJECT_NAME}_gencfg)
		endif()
		if (SRV_FILES)
			add_dependencies(${EXEC_NAME} ${PROJECT_NAME}_gencpp)
		endif()
	    if (ROS_GENERATE_MESSAGES)
	      add_dependencies(${EXEC_NAME} ${PROJECT_NAME}_generate_messages)
	    endif()
		# Specify libraries to link a library or executable target against
		target_link_libraries(${EXEC_NAME}
		  ${catkin_LIBRARIES}
		  ${mrt_LIBRARIES}
		)
		# Mark executables and/or libraries for installation
		install(TARGETS ${EXEC_NAME}
		  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)
	endif()
endfunction()
function(add_nodelet EXEC_NAME SEARCH_FOLDER)
	#glob all files in this directory
	file(GLOB EXEC_SOURCE_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${SEARCH_FOLDER}/*.h" "${SEARCH_FOLDER}/*.hpp")
	file(GLOB EXEC_SOURCE_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${SEARCH_FOLDER}/*.cpp")
    # Find nodelet
	file(GLOB NODELET_CPP RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${SEARCH_FOLDER}/*_nodelet.cpp")
	# Remove nodes (with their main) from src-files
	file(GLOB NODE_CPP RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${SEARCH_FOLDER}/*_node.cpp")
	if (NODE_CPP)
		list(REMOVE_ITEM EXEC_SOURCE_FILES_SRC ${NODE_CPP})
	endif ()
	if (NODELET_CPP)
        STRING(REGEX REPLACE "_node" "" NODELET_NAME ${EXEC_NAME})
        add_library(${NODELET_NAME}_nodelet
            ${EXEC_SOURCE_FILES_INC}
			${EXEC_SOURCE_FILES_SRC}
        )
        target_link_libraries(${NODELET_NAME}_nodelet
            ${catkin_LIBRARIES}
            ${mrt_LIBRARIES}
        )
        install(TARGETS ${NODELET_NAME}_nodelet
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
	set(NODELET_LIBRARIES ${NODELET_LIBRARIES} ${NODELET_NAME}_nodelet PARENT_SCOPE)
    endif()
endfunction()
glob_folders(SRC_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/src")
if (SRC_DIRECTORIES)
	#found subfolders, add executable for each subfolder
	foreach(SRC_DIR ${SRC_DIRECTORIES})
		add_exec(${SRC_DIR} "src/${SRC_DIR}")
		add_nodelet(${SRC_DIR} "src/${SRC_DIR}")
	endforeach()
else()
	#no subfolder found, add executable for src folder
	add_exec(${PROJECT_NAME} "src")
	add_nodelet(${PROJECT_NAME} "src")
endif()

#############
## Install ##
#############
#install(FILES
#  res/test.png
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
# Find and install nodelet plugin description file
file(GLOB PLUGINS_FILE RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "nodelet_plugins.xml")
if (PLUGINS_FILE)
    install(FILES nodelet_plugins.xml
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()

#############
## Testing ##
#############
# Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
	file(GLOB PROJECT_TEST_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test/*.cpp")
	foreach(PROJECT_TEST_FILE_SRC ${PROJECT_TEST_FILES_SRC})
		get_filename_component(PROJECT_TEST_NAME ${PROJECT_TEST_FILE_SRC} NAME_WE)
		#check for a .test file
		set(PROJECT_TEST_FILE_NAME_TEST "test/${PROJECT_TEST_NAME}.test")
		if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/${PROJECT_TEST_FILE_NAME_TEST})
			add_rostest_gtest(${PROJECT_TEST_NAME}-test ${PROJECT_TEST_FILE_NAME_TEST} ${PROJECT_TEST_FILE_SRC})
		else()
			catkin_add_gtest(${PROJECT_TEST_NAME}-test ${PROJECT_TEST_FILE_SRC})
		endif()
		target_link_libraries(${PROJECT_TEST_NAME}-test ${NODELET_LIBRARIES} ${catkin_LIBRARIES} ${mrt_LIBRARIES} gtest_main)
	endforeach()
endif()
