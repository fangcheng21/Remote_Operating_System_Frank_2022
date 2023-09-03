cmake_minimum_required(VERSION 3.2)
PROJECT(HAPTION)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${HAPTION_SOURCE_DIR}/bin)

INCLUDE_DIRECTORIES(
${HAPTION_SOURCE_DIR}/inc

)

#添加动态库目录
LINK_DIRECTORIES(${HAPTION_SOURCE_DIR}/lib)

add_executable(main ${HAPTION_SOURCE_DIR}/src/main.cpp )

TARGET_SOURCES(main 
	PRIVATE 
    ${HAPTION_SOURCE_DIR}/src/HaptionRobot.cpp   

	PUBLIC
     ${HAPTION_SOURCE_DIR}/inc/HaptionRobot.h
)  
  
target_link_libraries(main libvirtuose.a -lpthread -ldl)








