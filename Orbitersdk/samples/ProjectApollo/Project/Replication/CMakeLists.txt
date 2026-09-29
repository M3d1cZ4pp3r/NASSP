project(Replication)

set(INCLUDE_DIRS
    ${NASSP_SOURCE_ROOT}/src_sys
    ${NASSP_SOURCE_ROOT}/src_sys/replication
)
set(SOURCES
    ${NASSP_SOURCE_ROOT}/src_sys/replication/ReplicationCatalog.cpp
    ${NASSP_SOURCE_ROOT}/src_sys/replication/ReplicationHub.cpp
    ${NASSP_SOURCE_ROOT}/src_sys/replication/ReplicationTypes.cpp
)

add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIRS})
