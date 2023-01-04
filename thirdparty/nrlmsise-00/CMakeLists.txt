CMAKE_MINIMUM_REQUIRED(VERSION "2.8.8")

CMAKE_POLICY (SET "CMP0048" NEW)

PROJECT ("nrlmsise-00_builder" VERSION "" LANGUAGES "C" "CXX")

INCLUDE (ExternalProject)

EXTERNALPROJECT_ADD ("nrlmsise-00"
    GIT_REPOSITORY "https://github.com/magnific0/nrlmsise-00.git"
    CMAKE_ARGS 
    -DCMAKE_CXX_FLAGS=${MSVC_COMPILER_DEFS}
    UPDATE_DISCONNECTED 1
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
    INSTALL_COMMAND ""
)

# Specify include dir

EXTERNALPROJECT_GET_PROPERTY ("nrlmsise-00" source_dir)
SET (NRLMSISE_HEADER "${source_dir}/nrlmsise-00.h" PARENT_SCOPE)
SET (NRLMSISE_SRC "${source_dir}/nrlmsise-00.c" PARENT_SCOPE)

SET (NRLMSISE_LIBRARIES "${source_dir}/lib/libnrlmsise00.a" PARENT_SCOPE)
