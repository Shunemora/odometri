# odometri
simple odometri
# update the estimated position of the robot using it's wheel encoder readings
def _update_odometry( self ):
  R = self.robot_wheel_radius
  N = float( self.wheel_encoder_ticks_per_revolution )
  
  # read the wheel encoder values
  ticks_left, ticks_right = self.robot.read_wheel_encoders()
  
  # get the difference in ticks since the last iteration
  d_ticks_left = ticks_left - self.prev_ticks_left
  d_ticks_right = ticks_right - self.prev_ticks_right
  
  # estimate the wheel movements
  d_left_wheel = 2*pi*R*( d_ticks_left / N )
  d_right_wheel = 2*pi*R*( d_ticks_right / N )
  d_center = 0.5 * ( d_left_wheel + d_right_wheel )
  
  # calculate the new pose
  prev_x, prev_y, prev_theta = self.estimated_pose.scalar_unpack()
  new_x = prev_x + ( d_center * cos( prev_theta ) )
  new_y = prev_y + ( d_center * sin( prev_theta ) )
  new_theta = prev_theta + ( ( d_right_wheel - d_left_wheel ) / self.robot_wheel_base_length )
  
  # update the pose estimate with the new values
  self.estimated_pose.scalar_update( new_x, new_y, new_theta )
  
  # save the current tick count for the next iteration
  self.prev_ticks_left = ticks_left
  self.prev_ticks_right = ticks_right
