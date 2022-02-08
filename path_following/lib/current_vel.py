from math import sqrt

def get_current_vel(prev, current, dt):
    # prev = [이전 x, 이전 y, 이전 theta], current = [현재 x, 현재 y, 현재 theta]
    linear_velocity_x = (current[0] - prev[0]) / dt
    linear_velocity_y = (current[1] - prev[1]) / dt
    linear_velocity = sqrt(linear_velocity_x**2 + linear_velocity_y**2)
    angular_velocity = (current[2] - prev[2]) / dt
    
    return linear_velocity, angular_velocity