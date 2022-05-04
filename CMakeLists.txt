cmake_minimum_required( VERSION 2.8 )

project( AA-SIPP-m )

set(Boost_USE_STATIC_LIBS OFF) 
find_package(Boost COMPONENTS multi_index)
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( "." )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra" )
set( CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++" )

#Debug settings
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
#set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

set( SOURCE_FILES 
     main.cpp
     tinyxml2.cpp
     xmlLogger.cpp
     mission.cpp
     map.cpp
	 task.cpp
     config.cpp
	 dynamicobstacles.cpp
     aa_sipp.cpp
     constraints.cpp 
     ./Dubins/dubins.cpp)

set( HEADER_FILES 
	 primitive.h
     tinyxml2.h
     searchresult.h 
     gl_const.h 
     xmlLogger.h 
     mission.h 
     map.h 
	 task.h
	 dynamicobstacles.h
     config.h 
     aa_sipp.h 
     structs.h
     constraints.h
	 lineofsight.h 
     point.h
     dubins.h
     constants.h)
    
add_executable( AA-SIPP-m ${SOURCE_FILES} ${HEADER_FILES} )
