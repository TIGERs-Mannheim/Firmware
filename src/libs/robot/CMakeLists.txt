list(APPEND robot_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/ctrl.c
    ${CMAKE_CURRENT_LIST_DIR}/ctrl_motor.c
    ${CMAKE_CURRENT_LIST_DIR}/ctrl_panthera.c
    ${CMAKE_CURRENT_LIST_DIR}/ctrl_tigga.c
    ${CMAKE_CURRENT_LIST_DIR}/fusion_ekf.c
    ${CMAKE_CURRENT_LIST_DIR}/model_enc.c
    ${CMAKE_CURRENT_LIST_DIR}/model_fric.c
    ${CMAKE_CURRENT_LIST_DIR}/network.c
    ${CMAKE_CURRENT_LIST_DIR}/robot.c
    ${CMAKE_CURRENT_LIST_DIR}/skills.c
    ${CMAKE_CURRENT_LIST_DIR}/skill_basics.c
    ${CMAKE_CURRENT_LIST_DIR}/skill_circle_ball.c
    ${CMAKE_CURRENT_LIST_DIR}/skill_fast_pos.c
    ${CMAKE_CURRENT_LIST_DIR}/skill_get_ball.c
    ${CMAKE_CURRENT_LIST_DIR}/skill_intercept_ball.c
    ${CMAKE_CURRENT_LIST_DIR}/skill_penalty_shoot.c
    ${CMAKE_CURRENT_LIST_DIR}/skill_sine.c
    ${CMAKE_CURRENT_LIST_DIR}/skill_tc_2022.c
    ${CMAKE_CURRENT_LIST_DIR}/traj_bang_bang.c
    ${CMAKE_CURRENT_LIST_DIR}/util.c
)

list(APPEND robot_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/..)
