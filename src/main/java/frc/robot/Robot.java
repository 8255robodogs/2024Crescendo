package frc.robot;


import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.auto.programs.ExampleAuto;
import frc.robot.commands.auto.programs.TestAuto;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.subsystems.ExampleSys;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.VictorSPXMotorSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;


public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;
    private XboxController xbox0 = new XboxController(0);
    private XboxController xbox1 = new XboxController(1);
    
    private Command autonomousCommand;
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();
    


    //drivesystem
    private final SwerveSys swerveSys = new SwerveSys();
    private final ExampleSys exampleSys = new ExampleSys();


    //This is the top motor, running on its own
    private VictorSPXMotorSubsystem topMotor = new VictorSPXMotorSubsystem(9, "TopMotor", true);
    
    //These are the two motors that work together, they should be set very similarily
    private VictorSPXMotorSubsystem launcherMotorA = new VictorSPXMotorSubsystem(5, "LauncherMotorA", true);
    private VictorSPXMotorSubsystem launcherMotorB = new VictorSPXMotorSubsystem(6, "LauncherMotorB", true);


    //Ground pickup motor
    VictorSPXMotorSubsystem groundPickupMotor = new VictorSPXMotorSubsystem(3,"groundPickupMotor",true);

    //Launch feeder
    VictorSPXMotorSubsystem LauncherFeeder = new VictorSPXMotorSubsystem(2,"LauncherFeeder",false);


    //The two little servos for the camera gimbal. They are plugged into the PMW ports on the RIO.
    Servo servoCameraTurn = new Servo(9);
    Servo servoCameraPitch = new Servo( 8);
    
    PneumaticSubsystem lifter = new PneumaticSubsystem(2, 3, true);

    

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer(); //not using robotcontainer
        
        //create command selector for the smart dashboard and add Autos to it.
        SmartDashboard.putData("auto selector", autoSelector);
        autoSelector.setDefaultOption("Do Nothing", null);
        autoSelector.addOption("Example Auto", new TestAuto(swerveSys, launcherMotorA,launcherMotorB));
            
        //create the camera
        //UsbCamera camera = CameraServer.startAutomaticCapture();
        //camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);
        
        //create the swerve drive
        swerveSys.setSpeedFactor(.9);
        swerveSys.setDefaultCommand(new ArcadeDriveCmd(
            () -> MathUtil.applyDeadband(xbox0.getLeftY(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(xbox0.getLeftX(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(xbox0.getRightX(), ControllerConstants.joystickDeadband),
            true,
            true,
            swerveSys
        ));

        //set motor speeds
        topMotor.setMaxSpeed(1.0);

        launcherMotorA.setMaxSpeed(1.0);
        launcherMotorB.setMaxSpeed(1.0);
        launcherMotorA.SetSpeedAccelerationRate(1);
        launcherMotorB.SetSpeedAccelerationRate(1);
        launcherMotorA.SetSpeedFallRate(0);
        launcherMotorB.SetSpeedFallRate(0);

        servoCameraTurn.setAngle(90);
        servoCameraPitch.setAngle(90);
        servoCameraTurn.setPulseTimeMicroseconds(2500);
        servoCameraPitch.setPulseTimeMicroseconds(2500);
        
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
             
    }

    @Override
    public void autonomousInit() {
        //autonomousCommand = autoSelector.getSelected();
        autonomousCommand = new TestAuto(swerveSys, launcherMotorA,launcherMotorB);
        if(autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void teleopInit() {
        if(autonomousCommand != null) autonomousCommand.cancel();
        
    }

    @Override
    public void teleopPeriodic(){
        
        //DRIVER CONTROLLER (xbox0) is handled through the command system

        //Operator Controller (xbox1)

        //reset heading
        if(xbox1.getStartButtonPressed()){
            swerveSys.resetHeading();
        }

        //brakes
        if(xbox1.getLeftTriggerAxis() >0.3){
            swerveSys.lock();
        }


        //Launcher Feeder
        if(xbox1.getBButton() == true && xbox1.getXButton() == false){
            LauncherFeeder.SetSpeed(1);
            groundPickupMotor.SetSpeed(1);
        }else if(xbox1.getXButton() == true && xbox1.getBButton() == false){
            LauncherFeeder.SetSpeed(-.2);
            groundPickupMotor.SetSpeed(-0.2);
        }else{
            LauncherFeeder.SetSpeed(0);
            groundPickupMotor.SetSpeed(0);
        }


        //Launchers
        if(xbox1.getRightTriggerAxis() > 0.3 && xbox1.getLeftTriggerAxis() < 0.3){
            //only right trigger is pressed
            launcherMotorA.SetSpeed(1.0);
            launcherMotorB.SetSpeed(1.0);

        }else if(xbox1.getLeftTriggerAxis()> 0.3 && xbox1.getRightTriggerAxis() < 0.3){
            //only left trigger is pressed
            launcherMotorA.SetSpeed(-0.3);
            launcherMotorB.SetSpeed(-0.3);

        }else if (xbox1.getLeftTriggerAxis() < 0.3 && xbox1.getRightTriggerAxis() < 0.3){ 
            launcherMotorA.SetSpeed(0);
            launcherMotorB.SetSpeed(0);
        }


        //topmotor
        if(xbox1.getRightBumper()){
             topMotor.SetSpeed(0.6);
        }else if(xbox1.getLeftBumper()){
             topMotor.SetSpeed(-0.6);
        }else{
            topMotor.SetSpeed(0);
        }


        //camera rotation setup (this all needs to be made a subsystem some day)
        double cameraX = xbox1.getLeftX() *-1;
        double cameraY =  xbox1.getLeftY();
        if(Math.abs(cameraX) < .1){
            cameraX = 0;
        }
        if(Math.abs(cameraY) < .1){
            cameraY = 0;
        }
        cameraX = cameraX * 45;
        cameraY = cameraY * 45;
        cameraX = cameraX+90;
        cameraY=cameraY+90;
        cameraX = Math.round(cameraX);
        cameraY = Math.round(cameraY);
        servoCameraTurn.setAngle(cameraX);
        servoCameraPitch.setAngle(cameraY);
        //end of camera code


        //pneumatic liter --note we are using getButtonPRESSED, which is only true the moment the button is first pressed.
        if(xbox1.getRightStickButtonPressed()){
            lifter.TogglePneumatic();
        }


    }


    

}
