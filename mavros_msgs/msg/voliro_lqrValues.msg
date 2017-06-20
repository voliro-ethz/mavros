# lqr.msg
#
# send lqr pentalies to ros for saving in rosbag

# penalties on state
float32 q_state_ThetaX
float32 q_state_ThetaXdot
float32 q_state_ThetaY
float32 q_state_ThetaYdot
float32 q_state_PhiX
float32 q_state_PhiXdot
float32 q_state_PhiY
float32 q_state_PhiYdot

#penalties on integrator
float32 q_integrator_ThetaX
float32 q_integrator_ThetaY
float32 q_integrator_PhiX
float32 q_integrator_PhiY

#penalties on actuator inputs
float32 r_F_x
float32 r_F_y
float32 r_F_z
float32 r_T_x
float32 r_T_y
float32 r_T_z
