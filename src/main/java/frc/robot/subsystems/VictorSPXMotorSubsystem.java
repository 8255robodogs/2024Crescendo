package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VictorSPXMotorSubsystem extends SubsystemBase {
    
    private WPI_VictorSPX motor;
    private double currentSpeed = 0;
    private double speedFallRate = 0.06;
    private double speedAccelerationRate = 0.08;
    private double maxSpeed = 0.5;
    private String name;
    
    //This is an "instantiator". It governs the rules for what it takes to create an instance of this Subsystem.
    //In Robot.java or RobotContainer.java, I can create reuse this MotorSubsystem multiple times, creating several of them and giving them a different name and port number each time
    public VictorSPXMotorSubsystem(int motorPort, String motorName, boolean inverted){
        motor = new WPI_VictorSPX(motorPort);
        name = motorName;
        if(inverted==true){
            motor.setInverted(true);
        }
        

    }

    public Command AcceleratePositive(){

        return runOnce(
            () -> {
                currentSpeed = currentSpeed + speedAccelerationRate;
                
            }
        );


    }

    public Command AccelerateNegative(){
        return runOnce(
          () ->{
            currentSpeed = currentSpeed - speedAccelerationRate;

          }  
        );
    }



    //We can call this function to accelerate in one direction
    public void AccelerateInPositiveDirection(){
        currentSpeed = currentSpeed + speedAccelerationRate;
        
    }

    //We can call this function to accelerate in the other direction
    public void AccelerateInNegativeDirection(){
        currentSpeed = currentSpeed - speedAccelerationRate;
        
    }
    
    public void SetSpeed(double speed){
        currentSpeed =speed; 
    }

    public void SetSpeedFallRate(double rate){
        speedFallRate = rate;
    }

    public void SetSpeedAccelerationRate(double rate){
        speedAccelerationRate = rate;
    }

    public void ControlDirectly(double speed){
        
        if(Math.abs(speed) < 0.03){
            return;
        }

        
        currentSpeed = speed;
        if(currentSpeed > maxSpeed){
            currentSpeed = maxSpeed;
        }
        if(currentSpeed < -maxSpeed){
            currentSpeed = -maxSpeed;
        }

    }


    @Override
    public void periodic(){
        //code in here onstantly runs. 50 times per second i believe, so don't go wild with numbers.

        //Output data to the dashboard so we can monitor what is happening
        SmartDashboard.putNumber(name+" currentSpeed", currentSpeed);

        //make the speed falls constantly so the motor gradually comes to a stop.
        //make sure the speedAccelerationRate is greater than the speedFallRate so it can gain speed.
        //we're doing this to accelerate gradually rather than instant jerky motion.
        //Math.min gives us the lesser of 2 numbers. If currentSpeed was 0.1 and the fall rate is 0.4, we wouldn't want to overshoot and set the speed to -0.3
        if(currentSpeed > 0){
            //currentSpeed is positive, we need to subtract from currentSpeed
            currentSpeed -= Math.min(currentSpeed, speedFallRate);
        }else if(currentSpeed < 0){
            //currentSpeed is negative, we need to add to currentSpeed
            //Math.abs is short for absolute. It turns the number positive so we can correctly add it to our negative speed.
            currentSpeed += Math.min(Math.abs(currentSpeed), speedFallRate);
        }

        

        //speed clamping to respect the max speed
        if(currentSpeed > maxSpeed){
            currentSpeed=maxSpeed;
        }
        if(currentSpeed < -maxSpeed){
            currentSpeed = -maxSpeed;
        }

        

        //This is what actually controls the motor. It takes a number from -1 to 1. 0 is not moving.
        motor.set(VictorSPXControlMode.PercentOutput, currentSpeed);

    }

    public void setMaxSpeed(double speed){
        maxSpeed = speed;
    }

    public double getMaxSpeed(){
        return maxSpeed;
    }

    public double getCurrentSpeed(){
        return currentSpeed;
    }
   

}
