package frc.robot.commands.auto.programs;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.CmdSetMotorSpeed;
import frc.robot.commands.CmdSpinMotorNegative;
import frc.robot.commands.CmdSpinMotorPositive;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.commands.drivetrain.FollowTrajectoryCmd;
import frc.robot.commands.drivetrain.SetHeadingCmd;
import frc.robot.commands.drivetrain.SetPoseCmd;
import frc.robot.commands.drivetrain.SetTranslationCmd;

import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.VictorSPXMotorSubsystem;

public class AutoRedRight extends SequentialCommandGroup{

    public AutoRedRight(SwerveSys swerveSys, VictorSPXMotorSubsystem launcherMotorA, VictorSPXMotorSubsystem launcherMotorB, VictorSPXMotorSubsystem liftGate, 
    VictorSPXMotorSubsystem groundPickupMotor, VictorSPXMotorSubsystem launcherFeeder){
        PathPlannerPath path = PathPlannerPath.fromPathFile("Red Right Path");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("Red Right Path 2");
        PathPlannerPath path3 = PathPlannerPath.fromPathFile("Red Right Path 3");
        PathPlannerPath path4 = PathPlannerPath.fromPathFile("Red Right Path 4");

        addCommands(
            new CmdSpinMotorPositive(.75, launcherMotorA)
            .alongWith(new CmdSpinMotorPositive(.75, launcherMotorB)),
            new WaitCommand(0.1),
            new CmdSpinMotorPositive(.74, launcherFeeder),
            new CmdSetMotorSpeed(launcherMotorA,0),
            new CmdSetMotorSpeed(launcherMotorB, 0),

            new SetPoseCmd(new Pose2d(
                new Translation2d(15.78,4.34), new Rotation2d().fromDegrees(60)
            ), swerveSys),
            AutoBuilder.followPath(path),

            new CmdSpinMotorPositive(3, launcherFeeder)
            .alongWith(new CmdSpinMotorPositive(3,groundPickupMotor))
            .alongWith(AutoBuilder.followPath(path2))

            //new CmdSpinMotorNegative(.35, launcherFeeder)
            //.alongWith(new CmdSpinMotorNegative(.35,groundPickupMotor))
            //.alongWith(AutoBuilder.followPath(path3)),

            //AutoBuilder.followPath(path4),

            //new CmdSpinMotorPositive(.75, launcherMotorA)
            //.alongWith(new CmdSpinMotorPositive(.75, launcherMotorB)),
            //new WaitCommand(0.1),
            //new CmdSpinMotorPositive(.74, launcherFeeder),
            //new CmdSetMotorSpeed(launcherMotorA,0),
            //new CmdSetMotorSpeed(launcherMotorB, 0)
            //AutoBuilder.pathfindToPose(new Pose2d(2.9,7, new Rotation2d(0)),new PathConstraints(1,1,90,90))
        );
            
    }




}
