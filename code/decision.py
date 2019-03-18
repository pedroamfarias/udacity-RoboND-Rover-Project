import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 

            #Check if found some rock
            if Rover.rock_found:
                Rover.mode = 'get_rock'

            #Set Rover_stuck_str_time to first run
            elif Rover.vel < 0.2 and Rover.Rover_stuck_str_time == 0:
                Rover.Rover_stuck_str_time = Rover.total_time
                print("Set stuck_str_time as {}".format(Rover.Rover_stuck_str_time))
            
            #Check if Rover is stuck
            elif Rover.vel < 0.2 and (Rover.total_time - Rover.Rover_stuck_str_time) > 5:
                #Set Rover mode as stuck and rotate rover to try unstuck
                Rover.mode = 'stuck'
                print("Rover stuck for {} seconds".format(Rover.total_time - Rover.Rover_stuck_str_time))
                Rover.Rover_stuck_str_time = Rover.total_time

            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0

                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) # Force a bit to right
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif ( len(Rover.nav_angles) < Rover.stop_forward ): # and not Rover.rock_found :
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
                    print("Rover Stoped due to low nav_angles")
            elif Rover.yaw > 0:
                if Rover.yaw > 90:
                    if Rover.yaw > 180:
                        if Rover.yaw > 300:
                            Rover.rover_yaw_loop += 1
                            Rover.mode = 'stuck'
                            Rover.Rover_stuck_str_time = Rover.total_time   

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward :
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) #Force a bit to right
                    Rover.Rover_stuck_str_time = Rover.total_time # Update Rover_stuck_str_time to not go inside "stuck" loop
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code

        # If we're already in "stuck" mode then try to unstuck
        elif Rover.mode == 'stuck':
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # turn Rover for 2 seconds to try unstuck
                if (Rover.total_time - Rover.Rover_stuck_str_time) < 2:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                else:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 10) #Force a bit to right
                # else:
                    Rover.Rover_stuck_str_time = Rover.total_time
                    Rover.mode = 'forward'
                
                # Check if Rover start a 360º loop. If turn 360º set Rover mode as 'forward'
                if Rover.yaw > 0:
                    if Rover.yaw > 90:
                        if Rover.yaw > 180:
                            if Rover.yaw > 300:
                                Rover.rover_yaw_loop += 1
                                Rover.mode = 'forward'         

        # If we're already in "get_rock" mode then drive to Rock and try to get it
        elif Rover.mode == 'get_rock':
            print("Distance nav_dists: {}".format(np.mean(Rover.nav_dists)))
            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            if Rover.vel < Rover.max_vel:
                # Set throttle value to throttle setting
                Rover.throttle = Rover.throttle_set
            else: # Else coast
                Rover.throttle = 0
            Rover.brake = 0

            # If rock is not near, verify if Rover got stuck
            if not Rover.near_sample:
                if Rover.vel < 0.2 and Rover.Rover_stuck_str_time == 0:
                    Rover.Rover_stuck_str_time = Rover.total_time
                    print("Set stuck_str_time as {}".format(Rover.Rover_stuck_str_time))
                elif Rover.vel < 0.2 and (Rover.total_time - Rover.Rover_stuck_str_time) > 5: # and not Rover.rock_found:
                    Rover.mode = 'stuck'
                    print("Rover stuck for {} seconds".format(Rover.total_time - Rover.Rover_stuck_str_time))
                    Rover.Rover_stuck_str_time = Rover.total_time

            # If Rock is near, stop and get it
            if ( np.mean(Rover.nav_dists) < 20):
                if Rover.vel > 0.2:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                elif Rover.vel <= 0.2:
                    Rover.throttle = 0
                    #Rover.vel = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    # Rover near to rock and stoped: get the Rock!
                    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
                        Rover.send_pickup = True
                        Rover.rock_found = False
                        Rover.mode = 'forward'
    
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    
    return Rover

