package frc.robot.commands.auto.programs;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commands.drivetrain.SetTranslationCmd;

import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.VictorSPXMotorSubsystem;

public class TestAuto extends SequentialCommandGroup{

    public TestAuto(SwerveSys swerveSys, VictorSPXMotorSubsystem launcherMotorA, VictorSPXMotorSubsystem launcherMotorB, VictorSPXMotorSubsystem liftGate, 
    VictorSPXMotorSubsystem groundPickupMotor, VictorSPXMotorSubsystem launcherFeeder){
        //PathPlannerPath path = PathPlannerPath.fromPathFile("Short Path Forward");

        addCommands(
            new CmdSpinMotorNegative(1.5, liftGate),
            new CmdSpinMotorPositive(1, launcherMotorA)
            .alongWith(new CmdSpinMotorPositive(1, launcherMotorB)),
            new WaitCommand(2.5),
            new CmdSpinMotorPositive(1.5, launcherFeeder),
            new CmdSetMotorSpeed(launcherMotorA,0),
            new CmdSetMotorSpeed(launcherMotorB, 0),
            new SetHeadingCmd(new Rotation2d(-120), swerveSys),
            new SetTranslationCmd(new Translation2d(0.7, 6.8), swerveSys)
            //AutoBuilder.pathfindToPose(new Pose2d(2.9,7, new Rotation2d(0)),new PathConstraints(1,1,90,90))
            
            );
            
    }




}
