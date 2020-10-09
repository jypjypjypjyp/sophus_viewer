add_library(sophus_viewer SHARED
        viewer.cpp)

target_link_libraries(sophus_viewer ${THIRD_PARTY_LIBS})
target_compile_features(sophus_viewer PRIVATE cxx_std_14)