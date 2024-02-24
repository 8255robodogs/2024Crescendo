package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticSubsystem;

public class CmdTogglePneumatic extends CommandBase{
    private PneumaticSubsystem pneumatic;
    private boolean toggleDone = false;

    public CmdTogglePneumatic(PneumaticSubsystem pneumatic){
        this.pneumatic = pneumatic;
        addRequirements(pneumatic);
    }

    @Override
    public void initialize(){
        pneumatic.TogglePneumatic();
        toggleDone = true;
    }

    @Override 
    public void execute(){
        
    }

    @Override
    public boolean isFinished(){
        return toggleDone;
    }

    @Override
    public void end(boolean interrupted){
        
    }

 

}
