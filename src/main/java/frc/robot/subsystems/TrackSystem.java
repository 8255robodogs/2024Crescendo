package frc.robot.subsystems;

import javax.sound.midi.Track;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;

public class TrackSystem {
    
    private DigitalInput lowerLimitSwitch;
    private DigitalInput upperLimitSwitch;
    private VictorSPX motor;

    public TrackSystem(int motorID,int lowerLimitSwitchDIO,int upperLimitSwitchDIO){
        motor=new VictorSPX(motorID) ;
        lowerLimitSwitch = new DigitalInput(lowerLimitSwitchDIO);
        upperLimitSwitch = new DigitalInput(upperLimitSwitchDIO);
    }

    public void setSpeed(double speed){
        motor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public boolean getLowerLimitHit(){
        return lowerLimitSwitch.get();
    }

    public boolean getUpperLimitHit(){
        return upperLimitSwitch.get();
    }

}
