package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Servo;

public class CameraSystem {

    //Camera settings
    private final int cameraWidth = 320;
    private final int cameraHeight = 240;
    private final int cameraFPS = 15;

    //Two servos can be plugged into the PWM ports on the RoboRIO to allow turning and pitching the camera.
    Servo turnServo;
        private final int turnServoMicrosecondPulseTime = 2500;
        private final int turnServoMinValue = 0;
        private final int turnServoMaxValue = 180;
        private boolean turnServoInverted = false;

    Servo pitchServo;
        private final int pitchServoMicrosecondPulseTime = 2500;
        private final int pitchServoMinValue = 0;
        private final int pitchServoMaxValue = 180;
        private boolean pitchServoInverted = false;

    
    //Instantiator to create a simple camera.
    public CameraSystem(){
        initializeCamera();
    }

    //Instantiator to create a camera with turn and pitch servos.
    public CameraSystem(int turnServoPort, int pitchServoPort){
        this.turnServo = new Servo(turnServoPort);
        this.pitchServo = new Servo( pitchServoPort);
        initializeCamera();
    }

    //Instantiator to create a camera with turn and pitch servos, and immediately declare if servos should be inverted
    public CameraSystem(int turnServoPort, int pitchServoPort, boolean turnInverted, boolean pitchInverted){
        this.turnServo = new Servo(turnServoPort);
        this.pitchServo = new Servo( pitchServoPort);
        initializeCamera();
    }

    private void initializeCamera(){
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setVideoMode(PixelFormat.kMJPEG, cameraWidth, cameraHeight, cameraFPS);

        if(turnServo != null){
            turnServo.setPulseTimeMicroseconds(turnServoMicrosecondPulseTime);
            turnServo.setAngle((turnServoMinValue+turnServoMaxValue)/2); //set position to middle
        }
        if(pitchServo != null){
            pitchServo.setPulseTimeMicroseconds(pitchServoMicrosecondPulseTime);
            pitchServo.setAngle((pitchServoMinValue+pitchServoMaxValue)/2); //set positoin to middle
        }

    }

    //Expects two angles, probably something like a 0 to 180 value. Restricts the servo to obey the minimum and maximum angle values.
    public void setCameraAngle(double turnAngle, double pitchAngle){
        
        //invert the values if applicable
        if(turnServoInverted) turnAngle = turnServoMaxValue - turnAngle + turnServoMinValue;
        if(pitchServoInverted) pitchAngle = turnServoMaxValue - pitchAngle + pitchServoMinValue;

        //clamp the values within their ranges
        if(turnAngle < turnServoMinValue) turnAngle=turnServoMinValue;
        if(turnAngle > turnServoMaxValue) turnAngle=turnServoMaxValue;
        if(pitchAngle < pitchServoMinValue) pitchAngle=pitchServoMinValue;
        if(pitchAngle > pitchServoMaxValue) pitchAngle=pitchServoMaxValue;

        turnServo.setAngle(turnAngle);
        pitchServo.setAngle(pitchAngle);
    }

    //Expects a percentage (0 to 1.0) and sets the angle with respect to the servos minimum and maximum angles
    public void setCameraAngleByPercent(double turnPercent, double pitchPercent){
        //get the angle based on our minimum values, maximum values, and what percent through that range we are trying to reach
        double turnAngle = (((turnServoMaxValue - turnServoMinValue)*turnPercent)+turnServoMinValue);
        double pitchAngle = (((pitchServoMaxValue - pitchServoMinValue)*pitchPercent)+pitchServoMinValue);
        this.setCameraAngle(turnAngle, pitchAngle);
    }

    //Expects an axis from -1 to 1
    public void setCameraAngleByAxisValue(double turnValue, double pitchValue){
        
        //convert axis values (-1 to 1) to a percentage (0 to 1.0)
        double turnPercent = (turnValue +1)/2;
        double pitchPercent = (turnValue +1)/2;
        
        this.setCameraAngleByPercent(turnPercent, pitchPercent);
    }


    public void setTurnServoInverted(boolean inverted){
        this.turnServoInverted = inverted;
    }

    public void setPitchServoInverted(boolean inverted){
        this.pitchServoInverted = inverted;
    }

}
