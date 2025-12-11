idf_component_register(
    SRCS "ekf.cpp"
    INCLUDE_DIRS "include"
    # 关键：在这里声明依赖 Eigen
    # 因为 Eigen 是 managed component，通常只要 REQUIRES 就能找到
    REQUIRES eigen common
)