#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, MediumMotor, MoveTank, OUTPUT_A, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from time import sleep, time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.previous_error = 0
        self.integral = 0
        self.last_time = time()
        
    def calculate(self, error):
        current_time = time()
        dt = current_time - self.last_time
        
        # Prevent division by zero
        if dt <= 0:
            dt = 0.001
            
        # Calculate PID terms
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        
        # Calculate output
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        # Update state
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        self.previous_error = 0
        self.integral = 0
        self.last_time = time()

class FireTruck:
    def __init__(self):
        # Initialize motors
        self.left_motor = LargeMotor(OUTPUT_A)
        self.right_motor = LargeMotor(OUTPUT_B)
        self.injector_motor = MediumMotor(OUTPUT_C)
        self.drive_system = MoveTank(OUTPUT_A, OUTPUT_B)
        
        # Initialize sensors
        self.color_sensor_left = ColorSensor(INPUT_1)
        self.color_sensor_right = ColorSensor(INPUT_2)
        self.distance_sensor = UltrasonicSensor(INPUT_3)
        
        # Constants
        self.BASE_SPEED = 30
        self.LIGHT_THRESHOLD = 90
        self.INJECTION_SPEED = 50
        
        # PID Controllers
        # Tune these values based on your specific robot
        self.line_pid = PIDController(kp=0.5, ki=0.001, kd=0.1)
        self.injector_pid = PIDController(kp=0.8, ki=0.001, kd=0.2)
        
        # Injector parameters
        self.MIN_DISTANCE = 5  # cm
        self.MAX_DISTANCE = 50  # cm
        self.MAX_INJECTION_SPEED = 100
        self.water_velocity = 100  # cm/s (adjust based on your water pump)
        
    def calculate_injection_parameters(self, distance):
        """Calculate injection speed and time based on distance"""
        # Using projectile motion equations
        # Simplified model assuming straight line water trajectory
        
        # Calculate time needed for water to reach the target
        time_to_target = distance / self.water_velocity
        
        # Calculate required injection speed (percentage)
        # Linear mapping from distance to speed
        speed_percentage = ((distance - self.MIN_DISTANCE) / 
                          (self.MAX_DISTANCE - self.MIN_DISTANCE) * 
                          self.MAX_INJECTION_SPEED)
        
        # Constrain values
        speed_percentage = max(min(speed_percentage, self.MAX_INJECTION_SPEED), 0)
        
        return speed_percentage, time_to_target
        
    def check_for_fire(self):
        """Check all sensors for fire detection with PID control"""
        left_light = self.color_sensor_left.reflected_light_intensity
        right_light = self.color_sensor_right.reflected_light_intensity
        distance = self.distance_sensor.distance_centimeters
        
        # Calculate average light intensity
        avg_light = (left_light + right_light) / 2
        
        if (avg_light > self.LIGHT_THRESHOLD and 
            distance < self.MAX_DISTANCE and 
            distance > self.MIN_DISTANCE):
            return True, distance
        return False, 0
    
    def activate_injector(self, distance):
        """Activate water injector with PID-controlled speed"""
        print(f"Fire detected at {distance}cm! Activating water injector...")
        
        # Calculate initial injection parameters
        target_speed, injection_time = self.calculate_injection_parameters(distance)
        
        # Reset PID controller
        self.injector_pid.reset()
        
        # Extend injector with PID control
        start_time = time()
        while (time() - start_time) < injection_time:
            current_speed = self.injector_motor.speed
            error = target_speed - current_speed
            
            # Get PID output
            speed_adjustment = self.injector_pid.calculate(error)
            
            # Apply adjusted speed
            final_speed = target_speed + speed_adjustment
            final_speed = max(min(final_speed, self.MAX_INJECTION_SPEED), -self.MAX_INJECTION_SPEED)
            
            self.injector_motor.on(speed=final_speed)
            sleep(0.01)
            
        # Stop and retract injector
        self.injector_motor.off()
        self.injector_motor.on_for_seconds(speed=-self.INJECTION_SPEED, seconds=1)
        
    def follow_light_gradient(self):
        """Follow light gradient using PID control"""
        left_light = self.color_sensor_left.reflected_light_intensity
        right_light = self.color_sensor_right.reflected_light_intensity
        
        # Calculate error (difference between sensors)
        error = left_light - right_light
        
        # Get PID output
        steering = self.line_pid.calculate(error)
        
        # Apply steering with base speed
        left_speed = self.BASE_SPEED + steering
        right_speed = self.BASE_SPEED - steering
        
        # Constrain motor speeds
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)
        
        self.drive_system.on(left_speed, right_speed)
    
    def run(self):
        """Main control loop"""
        print("Fire truck is starting patrol with PID control...")
        
        while True:
            try:
                fire_detected, distance = self.check_for_fire()
                
                if fire_detected:
                    # Stop the truck
                    self.drive_system.off()
                    
                    # Activate water injector with calculated parameters
                    self.activate_injector(distance)
                    
                    # Reset PID controllers
                    self.line_pid.reset()
                    self.injector_pid.reset()
                    
                    # Wait before resuming patrol
                    sleep(2)
                    
                    print("Resuming patrol...")
                else:
                    # Continue following light gradient with PID control
                    self.follow_light_gradient()
                
                sleep(0.01)  # Small delay for sensor readings
                
            except Exception as e:
                print(f"Error occurred: {e}")
                self.drive_system.off()
                break

if __name__ == "__main__":
    # Create and run the fire truck
    fire_truck = FireTruck()
    fire_truck.run()
