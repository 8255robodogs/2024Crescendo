package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PneumaticSubsystem;

public class CmdSetPneumaticState extends Command{

    private PneumaticSubsystem pneumaticSubsystem;
    private boolean extended;

    public CmdSetPneumaticState(PneumaticSubsystem pneumaticSubsystem, boolean extended){
        this.pneumaticSubsystem = pneumaticSubsystem;
        this.extended = extended;
    }

    @Override
    public void initialize(){
        if(extended){        
            pneumaticSubsystem.PneumaticForward();
        }else{
            pneumaticSubsystem.PneumaticReverse();
        }
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




