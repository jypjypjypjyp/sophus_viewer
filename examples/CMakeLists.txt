add_executable(example example.cpp)
target_link_libraries(example ${THIRD_PARTY_LIBS} sophus_viewer)
target_compile_features(example PRIVATE cxx_std_14)