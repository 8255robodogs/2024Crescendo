package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@AutoLog
public class PneumaticSubsystem extends SubsystemBase{
    
    private DoubleSolenoid solenoid;

    public PneumaticSubsystem(int firstPort, int secondPort){
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, firstPort, secondPort);
        logSolenoidValue();
    }

    public void TogglePneumatic(){
        solenoid.toggle();
        logSolenoidValue();
    }

    public void PneumaticForward(){
        solenoid.set(Value.kForward);
        logSolenoidValue();
    }

    public void PneumaticReverse(){
        solenoid.set(Value.kReverse);
        logSolenoidValue();
    }

    public DoubleSolenoid.Value getSolenoidValue(){
        return solenoid.get();
    }

    private void logSolenoidValue(){
        Logger.recordOutput("Solenoid Value", solenoid.get());
    }

}