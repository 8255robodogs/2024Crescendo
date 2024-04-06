package frc.robot;


import java.text.DecimalFormat;
import java.util.Date;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.auto.programs.*;
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

    VictorSPXMotorSubsystem lifterGate = new VictorSPXMotorSubsystem(7, "lifter gate", true);

    //pneumatics subsystem
    PneumaticSubsystem pneumatics = new PneumaticSubsystem(2,3,false);

    DoubleLogEntry xlog;
    DoubleLogEntry ylog;
    DoubleLogEntry headingLog;
    DoubleArrayLogEntry poseLog;


    @Override
    public void robotInit() {
        robotContainer = new RobotContainer(); //not using robotcontainer
        
        // Starts recording to data log
        DataLogManager.start();
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLog log = DataLogManager.getLog();

        //create log entries to save data to
        xlog = new DoubleLogEntry(log, "/odometry/X");
        ylog = new DoubleLogEntry(log, "/odometry/Y");
        headingLog = new DoubleLogEntry(log, "/odometry/Heading");
        poseLog = new DoubleArrayLogEntry(log,"odometry/pose");

        //create command selector for the smart dashboard and add Autos to it.
        SmartDashboard.putData("auto selector", autoSelector);
        autoSelector.setDefaultOption("Do Nothing", null);
        autoSelector.addOption("Auto Shoot And Stay Still", new AutoShootAndStayStill(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));
        
        autoSelector.addOption("Auto Red Left", new AutoRedLeft(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));
        autoSelector.addOption("Auto Red Mid", new AutoShootAndStayStill(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));
        autoSelector.addOption("Auto Red Right", new AutoRedRight(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));

        autoSelector.addOption("Auto Blue Left (source side)", new AutoBlueLeft(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));
        autoSelector.addOption("Auto Blue Mid", new AutoBlueMiddle(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));
        autoSelector.addOption("Auto Blue Right (amp side)", new AutoBlueRight(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));

        

        //autoSelector.addOption("Example Auto", new TestAuto(swerveSys, launcherMotorA,launcherMotorB));
        //autoSelector.addOption("Test Auto", new TestAuto(swerveSys, launcherMotorA, launcherMotorB));    

        //create the camera
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);
        


        //create the swerve drive
        swerveSys.setSpeedFactor(1);
        swerveSys.setDefaultCommand(new ArcadeDriveCmd(
            () -> MathUtil.applyDeadband(xbox0.getLeftY(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(xbox0.getLeftX(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(xbox0.getRightX(), ControllerConstants.joystickDeadband),
            true,
            true,
            swerveSys
        ));

        

        launcherMotorA.setMaxSpeed(1); 
        launcherMotorB.setMaxSpeed(1); 
        launcherMotorA.SetSpeedAccelerationRate(1);
        launcherMotorB.SetSpeedAccelerationRate(1);
        launcherMotorA.SetSpeedFallRate(0);
        launcherMotorB.SetSpeedFallRate(0);

        
        

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        

        //writing data to the log
        xlog.append(swerveSys.getPose().getX());
        ylog.append(swerveSys.getPose().getY());
        headingLog.append(swerveSys.getPose().getRotation().getDegrees());
        poseLog.append(new double[]{swerveSys.getPose().getX(), swerveSys.getPose().getY(),swerveSys.getPose().getRotation().getDegrees()});
        
        //writing data to shuffleboard
        swerveSys.updateInterface();
    }

    @Override
    public void autonomousInit() {
        swerveSys.resetPose();
        //autonomousCommand = new TestAuto(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        
        //autonomousCommand = new AutoShootAndStayStill(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoBlueLeft(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoBlueMiddle(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoBlueRight(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoRedLeft(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoRedMiddle(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoRedRight(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        
        if(autoSelector.getSelected() != null){
            autonomousCommand = autoSelector.getSelected();
        }else{
            autonomousCommand = new AutoShootAndStayStill(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        }

        if(autonomousCommand != null) autonomousCommand.schedule(); //DONT TOUCH
    }

    @Override
    public void autonomousPeriodic(){

    }

    @Override
    public void teleopInit() {
        
        if(autonomousCommand != null) autonomousCommand.cancel();
        
    }

    @Override
    public void teleopPeriodic(){
        
        //DRIVER CONTROLLER (xbox0) driving is handled through the command system
        
        //reset heading
        if(xbox0.getStartButtonPressed()){
            swerveSys.resetHeading();
        }

        //brakes
        if(xbox0.getLeftTriggerAxis() >0.3){
            swerveSys.lock();
        }

        //hold RB to make the robot drive like a first person video game, instead of field relative.
        //This is useful for driving while looking at the onboard camera
        //NOT CURRENTLY WORKING, commenting out for now
        /*
        if(xbox0.getRightBumper() == true){
            swerveSys.setIsFieldOriented(true);
            swerveSys.setSpeedFactor(0.7);
        }else{
            swerveSys.setIsFieldOriented(false);
            swerveSys.setSpeedFactor(1);
        }
        */


        //Operator Controller (xbox1)

        //Launcher Feeder
        if(xbox1.getBButton() == true && xbox1.getXButton() == false){
            LauncherFeeder.SetSpeed(1);
            groundPickupMotor.SetSpeed(1);
        }else if(xbox1.getXButton() == true && xbox1.getBButton() == false){
            LauncherFeeder.SetSpeed(-.2);
            groundPickupMotor.SetSpeed(-0.2);    
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

        }else if(xbox1.getRightBumper()){
             launcherMotorA.SetSpeed(.5); //.525 CHANGE FOR AMP SCORING
             launcherMotorB.SetSpeed(0);  //.2 CHANGE FOR AMP SCORING
        }else if(xbox1.getLeftBumper()){
             launcherMotorA.SetSpeed(-0.6);
             launcherMotorB.SetSpeed(-.6);

        }else if (xbox1.getLeftTriggerAxis() < 0.3 && xbox1.getRightTriggerAxis() < 0.3){ 
            //neither triggers are pulled, stop the motors
            launcherMotorA.SetSpeed(0);
            launcherMotorB.SetSpeed(0);
        }


        
    
        //pneumatics
        if (xbox1.getLeftStickButtonPressed()){
            pneumatics.TogglePneumatic();
        }
        
        if(xbox1.getPOV() == 0){
            pneumatics.PneumaticReverse();
        }
        if(xbox1.getPOV() == 180){
            pneumatics.PneumaticForward();
        }


    }


    

}
