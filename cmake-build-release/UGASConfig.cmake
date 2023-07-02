include(CMakeFindDependencyMacro)

find_dependency(OpenCV 4.7 REQUIRED)

set(UGAS_PATH /home/alliance/Desktop/UGAS/release)

set(UGAS_LIBS  Control;Core;ThirdParty;Util)

set(UGAS_LINK_PATH ${UGAS_PATH}/lib)

set(UGAS_INCLUDE ${UGAS_PATH}/include)

include_directories(${UGAS_INCLUDE})

link_directories(${UGAS_LINK_PATH})

include("${CMAKE_CURRENT_LIST_DIR}/UGASTargets.cmake")

