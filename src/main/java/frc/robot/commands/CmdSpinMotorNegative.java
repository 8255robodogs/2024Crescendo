package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VictorSPXMotorSubsystem;

public class CmdSpinMotorNegative extends CommandBase{
    private VictorSPXMotorSubsystem motor;
    private double secondsToSpin;
    private long startTime;
    private float secondsElapsed = 0.0f;
    

    public CmdSpinMotorNegative( double seconds, VictorSPXMotorSubsystem motorSubsystem){
        secondsToSpin = seconds;
        this.motor = motorSubsystem;
        addRequirements(motorSubsystem);
        
    }

    @Override
    public void initialize(){
        startTime = System.nanoTime();
    }

    @Override 
    public void execute(){
        motor.AccelerateInNegativeDirection();
        
        //time tracking
        float currentTime = System.nanoTime();
        float nanoSecondsElapsed = currentTime - startTime;
        secondsElapsed = nanoSecondsElapsed / 1000000000; //divide by 1 billion

    }

    @Override
    public boolean isFinished(){
        //we have to return a true value at the end of isFinished to move onto the "end" function
        if(secondsElapsed >= secondsToSpin){
            return true;
        }else{
            return false;
        }
        
    }

    @Override
    public void end(boolean interrupted){
        
    }

 

}
