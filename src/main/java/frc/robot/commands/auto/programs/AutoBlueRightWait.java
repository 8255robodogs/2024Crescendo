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
import frc.robot.commands.drivetrain.SetPoseCmd;
import frc.robot.commands.drivetrain.SetTranslationCmd;

import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.VictorSPXMotorSubsystem;

public class AutoBlueRightWait extends SequentialCommandGroup{

    public AutoBlueRightWait(SwerveSys swerveSys, VictorSPXMotorSubsystem launcherMotorA, VictorSPXMotorSubsystem launcherMotorB, VictorSPXMotorSubsystem liftGate, 
    VictorSPXMotorSubsystem groundPickupMotor, VictorSPXMotorSubsystem launcherFeeder){
        PathPlannerPath path = PathPlannerPath.fromPathFile("Blue Wait");
        

        addCommands(
            new WaitCommand(1),
            //shoot first note
            new CmdSpinMotorPositive(.75, launcherMotorA)
            .alongWith(new CmdSpinMotorPositive(1, launcherMotorB)),
            new WaitCommand(0.1),
            new CmdSpinMotorPositive(.74, launcherFeeder),
            new CmdSetMotorSpeed(launcherMotorA,0),
            new CmdSetMotorSpeed(launcherMotorB, 0),

            new WaitCommand(10),

            //approach second note
            new SetPoseCmd(new Pose2d(
            new Translation2d(0.77,6.69), new Rotation2d().fromDegrees(-120)
            ), swerveSys),
            AutoBuilder.followPath(path)

            );
            
    }




}
