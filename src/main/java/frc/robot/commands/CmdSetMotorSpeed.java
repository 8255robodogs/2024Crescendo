package frc.robot.commands;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VictorSPXMotorSubsystem;

public class CmdSetMotorSpeed extends Command{
    private VictorSPXMotorSubsystem motor;
    private double speed;

    public CmdSetMotorSpeed(VictorSPXMotorSubsystem motor, double speed){
        this.motor = motor;
        this.speed = speed;
    }

    @Override
    public void initialize(){
        motor.SetSpeed(speed);
    }

    @Override 
    public void execute(){
        
        
        
    }

    @Override
    public boolean isFinished(){
        //we have to return a true value at the end of isFinished to move onto the "end" function
        return true;
        
    }

    @Override
    public void end(boolean interrupted){
        
    }

}




