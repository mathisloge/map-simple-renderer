cmake_minimum_required(VERSION 3.15)
file(READ vcpkg.json VCPKG_MANIFEST_STR)
string(JSON m_proj_ver GET ${VCPKG_MANIFEST_STR} "version-semver")
project(map-renderer VERSION ${m_proj_ver} LANGUAGES CXX)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GIT_REF)
message(STATUS "GIT_REF: ${GIT_REF}")
message(STATUS "GIT_REFSPEC: ${GIT_REFSPEC}")

include(GNUInstallDirs)

option(BUILD_SHARED_LIBS "build libraries as shared" ON)
