if(OpenCV_FOUND)
#agast
add_subdirectory(thirdparty/agast) 
include_directories(thirdparty/agast/include) 

file(GLOB BRISK_SOURCE_FILES  "src/brisk.cpp")

file(GLOB BRISK_HEADER_FILES  "include/brisk/brisk.h")

include_directories(include)

#build the brisk library dynamic and static versions
camodocal_library(camodocal_brisk SHARED ${BRISK_SOURCE_FILES} ${BRISK_HEADER_FILES})
camodocal_link_libraries(camodocal_brisk agast ${OpenCV_LIBS})

camodocal_install(camodocal_brisk)

# camodocal_executable(demo src/demo.cpp)
# camodocal_link_libraries(demo
#   ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
#   ${Boost_FILESYSTEM_LIBRARY}
#   ${Boost_PROGRAM_OPTIONS_LIBRARY}
#   ${Boost_SYSTEM_LIBRARY}
#   ${OpenCV_LIBS}
#   camodocal_brisk
# )

endif()
