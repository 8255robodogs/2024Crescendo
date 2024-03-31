package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }



    public static final class CANDevices {
        public static final int powerDistributionHubId = 0;

        public static final int imuId = 60;

        public static final int frontLeftCanCoderId = 18;
        public static final int frontLeftSteerMtrId = 12;
        public static final int frontLeftDriveMtrId = 15;

        public static final int frontRightCanCoderId = 28;
        public static final int frontRightSteerMtrId = 22;
        public static final int frontRightDriveMtrId = 25;

        public static final int backRightCanCoderId = 48;
        public static final int backRightSteerMtrId = 42;
        public static final int backRightDriveMtrId = 45;

        public static final int backLeftCanCoderId = 38;
        public static final int backLeftSteerMtrId = 32;
        public static final int backLeftDriveMtrId = 35;

        
    }

    public static final class ControllerConstants {
        public static final int driverGamepadPort = 0;

        public static final double joystickDeadband = 0.1;

        public static final double triggerPressedThreshhold = 0.25;
    }
    
    public static final class DriveConstants {
        
        public static final double trackWidth = Units.inchesToMeters(24);
        public static final double wheelBase = Units.inchesToMeters(24);

        /**
         * The SwerveDriveKinematics used for control and odometry.
         */
        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0),  // front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // back left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // back right
            );

        /**
         * The gear reduction from the drive motor to the wheel.
         * 
         * The drive gear ratios for the different levels can be found from the chart at
         * swervedrivespecialties.com/products/mk41-swerve-module.
         */
        // FIXME: This is the gear ratio for L3 modules.
        public static final double driveMtrGearReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        /**
         * The gear reduction from the steer motor to the wheel.
         */
        public static final double steerMtrGearReduction = (14.0 / 50.0) * (10.0 / 60.0);

        public static final double wheelRadiusMeters = Units.inchesToMeters(2);
        public static final double wheelCircumferenceMeters = 2.0 * wheelRadiusMeters * Math.PI;

        public static final double driveMetersPerEncRev = wheelCircumferenceMeters * driveMtrGearReduction;
        public static final double driveMetersPerSecPerRPM = driveMetersPerEncRev / 60.0;

        public static final double steerRadiansPerEncRev = 2 * Math.PI * DriveConstants.steerMtrGearReduction;

        public static final double kFreeMetersPerSecond = 5600 * driveMetersPerSecPerRPM;

        public static final double steerMtrMaxSpeedRadPerSec = 2.0;
        public static final double steerMtrMaxAccelRadPerSecSq = 1.0;

        public static final double maxDriveSpeedMetersPerSec = 5.0;

        /**
         * The rate the robot will spin with full Rot command.
         */
        public static final double maxTurnRateRadiansPerSec = 2.0 * Math.PI;

        public static final Rotation2d frontLeftModOffset = Rotation2d.fromDegrees(180 -20); 
        public static final Rotation2d frontRightModOffset = Rotation2d.fromDegrees(180 -15);
        public static final Rotation2d backRightModOffset = Rotation2d.fromDegrees(-80 -73); 
        public static final Rotation2d backLeftModOffset = Rotation2d.fromDegrees(180 -75);




        //public static final Rotation2d frontLeftModOffset = Rotation2d.fromDegrees((0.540283*360) -35 +180); 
        //public static final Rotation2d frontRightModOffset = Rotation2d.fromDegrees((0.391602*360) + 35.0);
        //public static final Rotation2d backRightModOffset = Rotation2d.fromDegrees((0.390869*360) +180 -112); 
        //public static final Rotation2d backLeftModOffset = Rotation2d.fromDegrees((0.834473*360) + 165);

        public static final int driveCurrentLimitAmps = 35;
        
        public static final double drivekP = 0.005;
        public static final double drivekD = 0.1;

        public static final double steerkP = 0.37431;
        public static final double steerkD = 0.27186;

        public static final double ksVolts = 0.667;
        public static final double kvVoltSecsPerMeter = 2.44;
        public static final double kaVoltSecsPerMeterSq = 0.0;

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(ksVolts, kvVoltSecsPerMeter, kaVoltSecsPerMeterSq);
    }

    public static final class AutoConstants {
        /**
         * The default maximum speed of the robot in auto. Can be overridden by the FollowTrajectoryCmd Command.
         */
        public static final double maxVelMetersPerSec = 3.25;

        public static final double drivekP = 1.2;
        public static final double drivekD = 1;

        public static final PIDConstants driveConstants = new PIDConstants(drivekD, drivekD);

        public static final double rotkP = 1.2;
        public static final double rotkD = 1;

        public static final PIDConstants rotConstants = new PIDConstants(rotkP, rotkD);
    }
}


