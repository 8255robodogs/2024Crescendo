package frc.robot.subsystems;

import java.text.DecimalFormat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Servo;

public class CameraSystem {

    //The two little servos for the camera gimbal. They are plugged into the PMW ports on the RIO.
    Servo servoCameraTurn = new Servo(9);
    Servo servoCameraPitch = new Servo( 8);
    
    DecimalFormat df = new DecimalFormat("###.###");

    public CameraSystem(){
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);
        
        servoCameraTurn.setAngle(90);
        servoCameraPitch.setAngle(90);
        servoCameraTurn.setPulseTimeMicroseconds(2500);
        servoCameraPitch.setPulseTimeMicroseconds(2500);
        
    }

}
