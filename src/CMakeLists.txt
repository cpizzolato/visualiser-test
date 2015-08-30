

include_directories(
    ${OPENSIM_INCLUDE_DIRS}
    ${SIMTK_INCLUDE_DIRS} )


ADD_EXECUTABLE(visualizerTest  main.cpp)
TARGET_LINK_LIBRARIES(visualizerTest
	  ${OPENSIM_LIBRARIES}
)

