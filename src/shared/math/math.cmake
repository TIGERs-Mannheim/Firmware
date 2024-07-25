list(APPEND math_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/abg_filter.c
    ${CMAKE_CURRENT_LIST_DIR}/angle_math.c
    ${CMAKE_CURRENT_LIST_DIR}/arm_atan2_f32.c
    ${CMAKE_CURRENT_LIST_DIR}/arm_mat_util_f32.c
    ${CMAKE_CURRENT_LIST_DIR}/clamp.c
    ${CMAKE_CURRENT_LIST_DIR}/ekf.c
    ${CMAKE_CURRENT_LIST_DIR}/ema_filter.c
    ${CMAKE_CURRENT_LIST_DIR}/fixed_point.c
    ${CMAKE_CURRENT_LIST_DIR}/hermite_spline.c
    ${CMAKE_CURRENT_LIST_DIR}/hermite_spline_cubic.c
    ${CMAKE_CURRENT_LIST_DIR}/kf.c
    ${CMAKE_CURRENT_LIST_DIR}/lag_element.c
    ${CMAKE_CURRENT_LIST_DIR}/line.c
    ${CMAKE_CURRENT_LIST_DIR}/luenberger.c
    ${CMAKE_CURRENT_LIST_DIR}/map_to_range.c
    ${CMAKE_CURRENT_LIST_DIR}/pi_ctrl_s12.c
    ${CMAKE_CURRENT_LIST_DIR}/pid.c
    ${CMAKE_CURRENT_LIST_DIR}/quat.c
    ${CMAKE_CURRENT_LIST_DIR}/signal_statistics.c
    ${CMAKE_CURRENT_LIST_DIR}/srukf.c
    ${CMAKE_CURRENT_LIST_DIR}/tracking_filter_2d.c
    ${CMAKE_CURRENT_LIST_DIR}/traj_1order.c
    ${CMAKE_CURRENT_LIST_DIR}/traj_2order.c
    ${CMAKE_CURRENT_LIST_DIR}/traj_generator.c
    ${CMAKE_CURRENT_LIST_DIR}/traj_minjerk.c
    ${CMAKE_CURRENT_LIST_DIR}/ukf.c
    ${CMAKE_CURRENT_LIST_DIR}/vector.c
)

list(APPEND math_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/..)

list(APPEND math_TEST_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/arm_atan2_f32.c
        ${CMAKE_CURRENT_LIST_DIR}/line.c
        ${CMAKE_CURRENT_LIST_DIR}/vector.c
)
