package frc.robot.commands;

import javax.sound.midi.Track;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrackSystem;
import frc.robot.subsystems.VictorSPXMotorSubsystem;

public class CmdTrackSystem extends Command{
    private TrackSystem trackSystem;
    private boolean goingUp;

    public CmdTrackSystem(TrackSystem trackSystem, boolean goingUp){
        this.trackSystem = trackSystem;
        this.goingUp = goingUp;
        
    }

    @Override
    public void initialize(){
        trackSystem.setSpeed(0.3);
    }

    @Override 
    public void execute(){
        
    }

    @Override
    public boolean isFinished(){
        //we have to return a true value at the end of isFinished to move onto the "end" function
        if(goingUp && trackSystem.getUpperLimitHit()){
            return true;
        }else if(goingUp == false && trackSystem.getLowerLimitHit()){
            return true;
        }else{
            return false;
        }
        
    }

    @Override
    public void end(boolean interrupted){
        trackSystem.setSpeed(0);
    }

 

}
