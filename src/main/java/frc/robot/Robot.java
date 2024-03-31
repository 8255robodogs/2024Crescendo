package frc.robot;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.auto.programs.*;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.VictorSPXMotorSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;


public class Robot extends LoggedRobot {
    
    private RobotContainer robotContainer;
    private XboxController xbox0 = new XboxController(0);
    private XboxController xbox1 = new XboxController(1);
    
    private Command autonomousCommand;
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    //drivesystem
    private final SwerveSys swerveSys = new SwerveSys();

    //This is the top motor, running on its own
    
    //These are the two motors that work together, they should be set very similarily
    private VictorSPXMotorSubsystem launcherMotorA = new VictorSPXMotorSubsystem(5, "LauncherMotorA", true);
    private VictorSPXMotorSubsystem launcherMotorB = new VictorSPXMotorSubsystem(6, "LauncherMotorB", true);

    //Ground pickup motor
    VictorSPXMotorSubsystem groundPickupMotor = new VictorSPXMotorSubsystem(3,"groundPickupMotor",true);

    //Launch feeder
    VictorSPXMotorSubsystem LauncherFeeder = new VictorSPXMotorSubsystem(2,"LauncherFeeder",false);

    VictorSPXMotorSubsystem lifterGate = new VictorSPXMotorSubsystem(7, "lifter gate", true);

    //pneumatics subsystem
    PneumaticSubsystem pneumatics = new PneumaticSubsystem(2,3);


    @Override
    public void robotInit() {
        Logger.recordMetadata("ProjectName", "2024Crescendo");
        // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
        Logger.start();
        
        robotContainer = new RobotContainer(); //not using robotcontainer
        //create command selector for the smart dashboard and add Autos to it.
        SmartDashboard.putData("auto selector", autoSelector);
        autoSelector.setDefaultOption("Do Nothing", null);
        autoSelector.addOption("Auto Shoot And Stay Still", new AutoShootAndStayStill(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));
        
        autoSelector.addOption("Auto Red Left", new AutoRedLeft(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));
        autoSelector.addOption("Auto Red Mid", new AutoShootAndStayStill(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));
        autoSelector.addOption("Auto Red Right", new AutoRedRight(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));

        autoSelector.addOption("Auto Blue Left", new AutoBlueLeft(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));
        autoSelector.addOption("Auto Blue Mid", new AutoBlueMiddle(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));
        autoSelector.addOption("Auto Blue Right", new AutoBlueRight(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));


        //autoSelector.addOption("Example Auto", new TestAuto(swerveSys, launcherMotorA,launcherMotorB));
        //autoSelector.addOption("Test Auto", new TestAuto(swerveSys, launcherMotorA, launcherMotorB));    

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
        swerveSys.updateInterface();
    }


    
    @Override
    public void autonomousInit() {
        swerveSys.resetPose(); //I'm not certain this is necessary, each autonomous should be setting the swerve system's pose anyway.
        

        //This block of code checks if the autonomous selector in Smart Dashboard has a routine selected.
        if(autoSelector.getSelected() != null){
            autonomousCommand = autoSelector.getSelected();
        }else{
            autonomousCommand = new AutoShootAndStayStill(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        }


        //These are all commented out. If something is wrong with the auto selection, we can uncomment one of these to hard-code which auto should run.
        //autonomousCommand = new TestAuto(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoShootAndStayStill(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoBlueLeft(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoBlueMiddle(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoBlueRight(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoRedLeft(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoRedMiddle(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        //autonomousCommand = new AutoRedRight(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder);
        
        

        if(autonomousCommand != null) autonomousCommand.schedule(); //DONT TOUCH. This is what actually schedules the auto command which was selected.
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

        //Operator Controller (xbox1)

        //Launcher Feeder
        if(xbox1.getBButton() == true && xbox1.getXButton() == false){
            LauncherFeeder.SetSpeed(1);
            groundPickupMotor.SetSpeed(1);
        }else if(xbox1.getXButton() == true && xbox1.getBButton() == false){
            LauncherFeeder.SetSpeed(-.2);
            groundPickupMotor.SetSpeed(-0.2);
            
        }else if(xbox0.getRightBumper() == true && xbox0.getLeftBumper() == false){
            LauncherFeeder.SetSpeed(1);
            groundPickupMotor.SetSpeed(1);

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

        }else if(xbox1.getRightBumper()){
             launcherMotorA.SetSpeed(0.525); //.525 CHANGE FOR AMP SCORING
             launcherMotorB.SetSpeed(.2);  //.2 CHANGE FOR AMP SCORING
        }else if(xbox1.getLeftBumper()){
             launcherMotorA.SetSpeed(-0.6);
             launcherMotorB.SetSpeed(-.6);

        }else if (xbox1.getLeftTriggerAxis() < 0.3 && xbox1.getRightTriggerAxis() < 0.3){ 
            launcherMotorA.SetSpeed(0);
            launcherMotorB.SetSpeed(0);
        }


        //pneumatics
        if (xbox1.getLeftStickButtonPressed()){
            pneumatics.TogglePneumatic();
        }
        SmartDashboard.putNumber("xbox1hat", xbox1.getPOV());
        if(xbox1.getPOV() == 0){
            pneumatics.PneumaticReverse();
        }
        if(xbox1.getPOV() == 180){
            pneumatics.PneumaticForward();
        }


    }


    

}
