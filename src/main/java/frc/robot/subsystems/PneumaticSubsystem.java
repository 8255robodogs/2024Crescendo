package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PneumaticSubsystem extends SubsystemBase{
    
    private DoubleSolenoid solenoid;

    public PneumaticSubsystem(){

    }

    public PneumaticSubsystem(int firstPort, int secondPort){
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, firstPort, secondPort);
    }

    public void TogglePneumatic(){
        solenoid.toggle();
    }

    public void PneumaticForward(){
        solenoid.set(Value.kForward);
    }

    public void PneumaticReverse(){
        solenoid.set(Value.kReverse);
    }

    public DoubleSolenoid.Value getSolenoidValue(){
        return solenoid.get();
    }

    

}