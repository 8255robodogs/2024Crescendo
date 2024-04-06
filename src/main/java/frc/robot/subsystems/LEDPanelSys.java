package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDPanelSys extends SubsystemBase {
    
    AddressableLED strip;
    AddressableLEDBuffer buffer;

    int cycle = 0;
    int cycleTime=20;

    

    public LEDPanelSys(int PMWport){
        strip.setLength(256);
        strip.setData(buffer);
        strip.start();

    }

    public void setAllToColor(Color color){
        for(int i=0;i<buffer.getLength();i++){
            buffer.setLED(i, color);
        }
    }




}
