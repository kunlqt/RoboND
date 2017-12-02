import numpy as np
import random

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    # near_sample property seems to stay asserted for a short while even after pickup is completed
    # this can cause this loop to be selected for ever and get stuck
    # so add in a timeout after pickup has occured until next one is allowed to happen
    
    #Record starting point of rover so we can return there after all samples collected
    if Rover.count == 0:
        Rover.start_pos = Rover.pos

    #if all samples are found, return to starting location
    
    #print("DEBUG - Rover.samples_found", Rover.samples_found)
    if np.all(Rover.samples_found == [1,1,1,1,1,1]) and distance_to_start < Rover.close_to_goal_threshold:
    #if Rover.number_samples_collected == 6:
        #print("DEBUG - All samples found, returning home")
        #print("DEBUG - start_pos:", Rover.start_pos)
        #print("DEBUG - current_post", Rover.pos)

        #rover_to_start = np.array([Rover.start_pos[0] - Rover.pos[0], Rover.start_pos[1] - Rover.pos[1]])
        #distance_to_start = (rover_to_start[0]**2 + rover_to_start[1]**2)**0.5

        #if close to goal then challenge complete!
        #if distance_to_start < Rover.close_to_goal_threshold:
        print("DEBUG - Congrats, challenge completed!")
        Rover.throttle = 0
        Rover.brake = Rover.brake_set

        #TODO - code to actively navigate towards starting point

        # else:
        #     yaw_rad = Rover.yaw * np.pi / 180

        #     dummy_r_x = Rover.pos[0] * np.cos(yaw_rad) / 0.5**0.5
        #     dummy_r_y = Rover.pos[1] * np.sin(yaw_rad) / 0.5**0.5
        #     rover_to_dummy_r = np.array([dummy_r_x - Rover.pos[0], dummy_r_y - Rover.pos[1]])

        #     dot = np.dot(rover_to_start, rover_to_dummy_r)
        #     angle_to_turn = np.arccos(dot / distance_to_start)

        #     Rover.steer = angle_to_turn
        #     Rover.throttle = Rover.throttle_set
        #     Rover.brake = 0


    #if near the rock (and not in timeout) pick it up
    elif Rover.near_sample and Rover.count > Rover.near_sample_count + Rover.timeout_after_pickup:
        Rover.near_sample_count = Rover.count

        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        
        #pick up the rock
        #if Rover.vel == 0:
        Rover.pick_up = True

    #Check if there are any rocks visible
    elif len(Rover.rocks_angles) > 0:
        #print("DEBUG - found rocks")
        #print(Rover.rocks_angles)
        #print(Rover.rocks_dists)
        
        #if there is a rock, move *slowly* directly towards it
        Rover.throttle = 0.2
        Rover.steer = np.clip(np.mean(Rover.rocks_angles * 180/np.pi), -15, 15)



    # Check if we have vision data to make decisions with
    elif Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                if random.random() > 0.01:
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi) +15, -15, 15)
                    #Rover.steer = np.clip(np.percentile(Rover.nav_angles * 180/np.pi, 75), -15, 15) #aim to make wall-follower
                else:
                    Rover.mode = 'random'
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

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
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -10 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi) + 15, -15, 15)
                    Rover.mode = 'forward'

        elif Rover.mode == 'random':
            if random.random() > 0.95:
                Rover.mode = 'forward'
            else:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = -10

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

