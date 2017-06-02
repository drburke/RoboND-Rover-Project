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

        # Occasionally Rover.nav_angles is empty
        if len(Rover.nav_angles) == 0:
            Rover.nav_angles = np.array([0.])
            Rover.nav_dists = np.array([1.])


        #  If picking up, set to 'stuck' mode to initiate reversal once pickup is done
        dist_to_start = np.sqrt((Rover.pos[0] - Rover.initial_x)**2 + (Rover.pos[1] - Rover.initial_y)**2)

        if Rover.rock_count >= Rover.total_rocks and not Rover.picking_up and dist_to_start < 3:
            Rover.brake = Rover.brake_set
            Rover.throttle = 0

        elif Rover.picking_up:
            Rover.worldmap[range(np.int_(Rover.pos[1])-5,np.int_(Rover.pos[1])+6), range(np.int_(Rover.pos[0])-5,np.int_(Rover.pos[0])+6), 1] += 1
            Rover.mode = 'stuck'
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
            Rover.steer = 0
            Rover.stuck_counter = 21
            if ~Rover.near_sample:
                Rover.throttle

        elif Rover.near_sample:  # if near samples stop

            if Rover.mode == 'forward':
                Rover.mode = 'forward'
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.steer = 0
                if Rover.vel==0:
                    Rover.mode = 'stop'
            elif Rover.mode == 'stop':  # if near sample and stopped, pickup
                Rover.worldmap[np.int_(Rover.pos[1]), np.int_(Rover.pos[0]), 1] += 1
                Rover.send_pickup = True
                Rover.rock_count += 1


        elif Rover.mode == 'stuck':  # if stuck, intiate reversal and counter
            
            Rover.stuck_counter += 1
            if Rover.stuck_counter > 20:
                Rover.throttle = -0.5
                Rover.steer = -np.clip(np.mean(Rover.nav_angles * 180/np.pi)+25, -15, 15)
            else:
                Rover.throttle = 0
                Rover.steer = 15
            Rover.brake = 0
            
            if Rover.stuck_counter > 80:
                Rover.mode = 'stop'
                Rover.stuck_counter = 0
                Rover.brake = 10.
                Rover.throttle = 0.

        elif len(Rover.rock_angles) > 0:  # if we see a rock, move towards it slowly
            if Rover.mode == 'forward': 
                # Check the extent of navigable terrain
                if Rover.throttle > 0. and Rover.vel <= 0.1:
                    Rover.stuck_counter += 1
                    if Rover.stuck_counter > 200:
                        Rover.mode = 'stuck'
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = 15
                        Rover.stuck_counter = 0
            if Rover.mode != 'stuck':
                if Rover.vel > 0.5:
                    Rover.brake = 10.
                    Rover.throttle = -0.1
                else:
                    Rover.brake = 0
                    Rover.throttle = 0.4
                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
                Rover.mode = 'forward'
                Rover.brake = 0



        # Check for Rover.mode status
        elif Rover.mode == 'forward': 
            # Check the extent of navigable terrain

            if len(Rover.close_obs) > 50:
                # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 15
                    Rover.mode = 'stop'

            elif Rover.throttle > 0. and Rover.vel <= 0.1:
                Rover.stuck_counter += 1
                if Rover.stuck_counter > 200:
                    Rover.mode = 'stuck'
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = 15
                    Rover.stuck_counter = 0

            elif len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                
                # Set steering to average angle clipped to the range +/- 15

                alpha = 0.8
                # close_indices = Rover.nav_dists < 5.
                # close_angles = Rover.nav_angles[np.where(Rover.nav_dists < 20.)]
                # print(close_angles)
                #new_angle = np.average(Rover.heat_angles,weights=(Rover.heat * (np.max(Rover.nav_dists) - Rover.nav_dists))) * 180./np.pi - 15
                new_angle = np.average(Rover.heat_angles,weights=Rover.heat) * 180./np.pi - 10
                #new_angle = Rover.heat_angles[np.argmax(Rover.heat)]
                steer_angle = new_angle * alpha 

                max_vel = Rover.max_vel
                Rover.brake = 0
                if np.abs(steer_angle) > 2.:
                    max_vel = 1.
                    Rover.brake = 0.
                elif np.abs(steer_angle) > 4.:
                    max_vel = 0.2
                    Rover.brake = 1.0
                elif np.abs(steer_angle) >= 8.:
                    max_vel = -999.
                    Rover.brake = Rover.brake_set


                if Rover.vel < max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                

                #Rover.heat_angles[np.argmax(Rover.heat)] * alpha * 180./np.pi -4
                # if len(Rover.obs_dists) > 0 and np.min(Rover.obs_dists) > 20:
                #     steer_angle = Rover.heat_angles[np.argmax(Rover.heat)] * alpha * 180./np.pi -4
                # else:
                #     steer_angle = Rover.heat_angles[np.argmax(Rover.heat)] * alpha * 180./np.pi -4
                    #steer_angle = np.percentile(Rover.nav_angles,20.)*alpha * 180./np.pi +2.
                #steer_angle = np.min(Rover.nav_angles)*alpha * 180./np.pi + 6.
                Rover.steer = np.clip(steer_angle,-15,15) #np.clip(np.mean(close_angles * 180/np.pi -5)*alpha, -15, 15)
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
                if len(Rover.nav_angles) < Rover.go_forward or (len(Rover.close_obs) > 50):
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
        else:
            Rover.mode = 'stuck'
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
            Rover.steer = 0
            Rover.stuck_counter = 0
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover

