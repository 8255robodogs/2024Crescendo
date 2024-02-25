package frc.robot.commands.auto.programs;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.CmdSpinMotorNegative;
import frc.robot.commands.auto.FollowTrajectoryCmd;
import frc.robot.commands.drivetrain.SetTranslationCmd;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.VictorSPXMotorSubsystem;

public class TestAuto extends SequentialCommandGroup{

    public TestAuto(SwerveSys swerveSys, VictorSPXMotorSubsystem launcherMotorA, VictorSPXMotorSubsystem launcherMotorB){
        addCommands(
            
            new CmdSpinMotorNegative(1, launcherMotorA)
            .alongWith(new CmdSpinMotorNegative(1, launcherMotorB)),
            new WaitCommand(1.5)
        );
    }




}
