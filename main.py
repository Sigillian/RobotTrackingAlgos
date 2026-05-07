from math import sin, pi
from math import cos

### ASSUMPTIONS MADE ###
# zero slippage with tracking wheels and bot
# perfect encoder resolution
# distances provided are in actual distances, not ticks ((2*pi*tracking_wheel_radius*ticks)/ticks_per_revolution)
# forward is X
# lateral is Y
# +x forward
# +y left
# +angle is counterclockwise

#DIFFERENTIAL DRIVE ONLY -- WILL NOT WORK WITH MECANUM WHEELS
# |````````=====````````|
# |          L          |
# |                     |      ^
# |          O          |      | +Y
# |                     |
# |          R          |
# |,,,,,,,,=====,,,,,,,,|
#            ->
#            +X
# O is the tracking center of the bot
def differential_two_wheel_tracker (last_left, last_right, current_left, current_right, track_width, last_x, last_y, last_angle) -> list[float]:
    #changes in tracking wheel movements since last update
    delta_right = current_right - last_right
    delta_left = current_left - last_left

    #change in heading
    delta_angle = (delta_right - delta_left)/track_width

    if abs(delta_angle) > 1e-7:
        #algebraically most simple way of computing IRC for center, intermediary variables were compressed
        instantaneous_radius_of_curvature_center = ((delta_right + delta_left) * track_width) / ((delta_right - delta_left) * 2)
    else:
        #EDGE CASE: in case of straight forward motion where delta_right = delta_left, the before code would have caused a division by zero.
        global_delta_x = ((delta_right + delta_left) / 2) * cos(last_angle)
        global_delta_y = ((delta_right + delta_left) / 2) * sin(last_angle)
        return [last_x + global_delta_x, last_y + global_delta_y, last_angle]

    #
    delta_x = instantaneous_radius_of_curvature_center * sin(delta_angle)
    delta_y = instantaneous_radius_of_curvature_center * (1 - cos(delta_angle))

    #so far we've only computed the localized (update-specific) coordinate changes
    return global_transform(last_x, last_y, last_angle, delta_x, delta_y, delta_angle)

#THIS WORKS WITH MECANUM WHEELS! Three tracking wheels are required in the following configuration:
# |````````=====````````|
# |          L          |
# #                     |      ^
# # B        O          |      | +Y
# #                     |
# |          R          |
# |,,,,,,,,=====,,,,,,,,|
#            ->
#            +X
# O is the tracking center of the bot
def holonomic_three_wheel_tracker (last_left, last_right, last_back, current_left, current_right, current_back, track_width, back_wheel_to_center, last_x, last_y, last_angle) -> list[float]:
    #changes in tracking wheel movements since last update
    delta_right = current_right - last_right
    delta_left = current_left - last_left
    delta_back = current_back - last_back

    #same as differential drive
    delta_angle = (delta_right - delta_left) / track_width

    #x is forward movement
    delta_x_body = (delta_left + delta_right) / 2
    #y is lateral movement (back_wheel_to_center should be positive. otherwise, calculate as ``delta_back - (back_wheel_to_center * delta_angle)``)
    delta_y_body = delta_back + (back_wheel_to_center * delta_angle)

    #globalize, same as differential
    return global_transform(last_x, last_y, last_angle, delta_x_body, delta_y_body, delta_angle)


#turn localized (update-specific) coordinate changes to global ones based on the current position
def global_transform (last_x, last_y, last_angle, delta_x, delta_y, delta_angle):
    #new angle is just the sum of the old and new angle
    final_angle = last_angle + delta_angle

    #simple rotational matrix
    global_delta_x = delta_x * cos(last_angle) - delta_y * sin(last_angle)
    global_delta_y = delta_x * sin(last_angle) + delta_y * cos(last_angle)

    #sum the change and the old for final values
    final_x = global_delta_x + last_x
    final_y = global_delta_y + last_y

    return [final_x, final_y, final_angle]


print("Reminder: angles are in radians")
print("Straight forward test")
print("Holonomic: " + str(holonomic_three_wheel_tracker(0, 0, 0, 100, 100, 0, 10, 10, 0, 0, 0)))
print("Differential: " + str(differential_two_wheel_tracker(0,0,100,100,10,0,0,0)))
print()
print("Strafe test")
print("Holonomic: " + str(holonomic_three_wheel_tracker(0, 0, 0, 100, 100, 10, 10, 10, 0, 0, 0)))
print()
print("Turn test (90 deg. || 1.57 rad.)")
print("Holonomic: " + str(holonomic_three_wheel_tracker(0, 0, 0, -7.85398, 7.85398, -7.85398, 10, 5, 0, 0, 0)))
print("Differential: " + str(differential_two_wheel_tracker(0,0,-7.85398, 7.85398,10,0,0,0)))