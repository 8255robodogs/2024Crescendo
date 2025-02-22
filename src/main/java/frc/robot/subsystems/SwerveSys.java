package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class SwerveSys extends SubsystemBase {

    // Initializes swerve module objects
    private final SwerveModule frontLeftMod = 
        new SwerveModule(
            CANDevices.frontLeftDriveMtrId,
            CANDevices.frontLeftSteerMtrId,
            CANDevices.frontLeftCanCoderId,
            DriveConstants.frontLeftModOffset
        );

    private final SwerveModule frontRightMod = 
        new SwerveModule(
            CANDevices.frontRightDriveMtrId,
            CANDevices.frontRightSteerMtrId,
            CANDevices.frontRightCanCoderId,
            DriveConstants.frontRightModOffset
        );

    private final SwerveModule backRightMod = 
        new SwerveModule(
            CANDevices.backRightDriveMtrId,
            CANDevices.backRightSteerMtrId,
            CANDevices.backRightCanCoderId,
            DriveConstants.backRightModOffset
        );

    private final SwerveModule backLeftMod = 
        new SwerveModule(
            CANDevices.backLeftDriveMtrId,
            CANDevices.backLeftSteerMtrId,
            CANDevices.backLeftCanCoderId,
            DriveConstants.backLeftModOffset
        );

    private boolean isLocked = false;
    public boolean isLocked() {
        return isLocked;
    }

    private boolean isFieldOriented = false;
    public boolean isFieldOriented() {
        return isFieldOriented;
    }
    public void setIsFieldOriented(boolean bool){
        this.isFieldOriented=bool;
    }

    private double speedFactor = 1.0;
    public double getSpeedFactor() {
        return speedFactor;
    }
    /**
     * Sets the speed factor of the robot. Inputs are multiplied by this factor to reduce drive speed.
     * Useful for "turtle" or "sprint" modes.
     * @param speedFactor The factor to scale inputs, as a percentage.
     */
    public void setSpeedFactor(double speedFactor) {
        this.speedFactor = speedFactor;
    }

    private final Pigeon2 imu = new Pigeon2(CANDevices.imuId);

    // Odometry for the robot, measured in meters for linear motion and radians for rotational motion
    // Takes in kinematics and robot angle for parameters

    private SwerveDrivePoseEstimator odometry = 
        new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d()
        );

    /**
     * Constructs a new SwerveSys.
     * 
     * <p>SwerveCmd contains 4 {@link SwerveModule}, a gyro, and methods to control the drive base and odometry.
     */
    public SwerveSys() {
        // Resets the measured distance driven for each module
        frontLeftMod.resetDriveDistance();
        frontRightMod.resetDriveDistance();
        backLeftMod.resetDriveDistance();
        backRightMod.resetDriveDistance();

        frontLeftMod.invertSteerMtr();
        frontRightMod.invertSteerMtr();
        backLeftMod.invertSteerMtr();
        backRightMod.invertSteerMtr();

        resetPose();


    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::setPose, 
      this::getChassisSpeeds, 
      this::setChassisSpeeds, 
      new HolonomicPathFollowerConfig(
      AutoConstants.driveConstants, // Translation constants 
      AutoConstants.rotConstants, // Rotation constants 
      1, 
      Units.inchesToMeters(16.97), // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    ),
      () -> {
        /* 
        var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          */
          return false;
      },
      this
    );

    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // Updates the odometry every 20ms
        odometry.update(getHeading(), getModulePositions());
    }
    
    /**
     * Inputs drive values into the swerve drive base.
     * 
     * @param driveX The desired forward/backward lateral motion, in meters per second.
     * @param driveY The desired left/right lateral motion, in meters per second.
     * @param rotation The desired rotational motion, in radians per second.
     * @param isFieldOriented whether driving is field- or robot-oriented.
     */
    public void drive(double driveX, double driveY, double rotation, boolean isFieldOriented) {  
        if(driveX != 0.0 || driveY != 0.0 || rotation != 0.0) isLocked = false;
        
        if(isLocked) {
            setModuleStatesOpenLoop(new SwerveModuleState[] {
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI))
            });
        }
        else {
            // Reduces the speed of the drive base for "turtle" or "sprint" modes.
            driveX *= speedFactor;
            driveY *= speedFactor;
            rotation *= speedFactor;

            // Represents the overall state of the drive base.
            ChassisSpeeds speeds =
            isFieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveX, driveY, rotation, getHeading())
                : new ChassisSpeeds(driveX, driveY, rotation);

            // Uses kinematics (wheel placements) to convert overall robot state to array of individual module states.
            SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);
            
            // Makes sure the wheels don't try to spin faster than the maximum speed possible
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxDriveSpeedMetersPerSec);

            setModuleStatesOpenLoop(states);
        }
    }

    /**
     * Stops the driving of the drive base.
     * <p>Sets all drive inputs to zero. This will set the drive power of each module to zero while maintaining module headings.
     */
    public void stop() {
        drive(0.0, 0.0, 0.0, isFieldOriented);
    }

    /**
     * Turns the modules to the X-lock position as long as drive inputs are zero.
     */
    public void lock() {
        isLocked = true;
    }

    /**
     * Sets the desired state for each swerve module.
     * <p>Controls the lienar and rotational values for the modules based on the free speed of the drive motors (open-loop).
     * 
     * @param moduleStates An array of module states to set. The order is FL, FR, BL, BR.
     */
    public void setModuleStatesOpenLoop(SwerveModuleState[] moduleStates) {
        frontLeftMod.setDesiredState(moduleStates[0], false);
        frontRightMod.setDesiredState(moduleStates[1], false);
        backRightMod.setDesiredState(moduleStates[3], false);
        backLeftMod.setDesiredState(moduleStates[2], false);
    }

    /**
     * Sets the desired state for each swerve module.
     * <p>Uses PID and feedforward control (closed-loop) to control the linear and rotational values for the modules.
     * 
     * @param moduleStates An array module states to set. The order is FL, FR, BL, BR.
     */
    public void setModuleStatesClosedLoop(SwerveModuleState[] moduleStates) {
        frontLeftMod.setDesiredState(moduleStates[0], true);
        frontRightMod.setDesiredState(moduleStates[1], true);
        backRightMod.setDesiredState(moduleStates[3], true);
        backLeftMod.setDesiredState(moduleStates[2], true);
    }

    /**
     * Returns the current motion of the drive base as a ChassisSpeeds.
     * 
     * @return A ChassisSpeeds representing the current motion of the drive base.
     */
    public ChassisSpeeds getChassisSpeeds() {
        double xVel = getAverageDriveVelocityMetersPerSec() * getDirectionOfTravel().getCos();
        double yVel = getAverageDriveVelocityMetersPerSec() * getDirectionOfTravel().getSin();
        double omega = Units.degreesToRadians(-imu.getRate());

        return new ChassisSpeeds(xVel, yVel, omega);
    }

    /**
     * Sets the ChassisSpeeds of the drive base.
     * 
     * @param chassisSpeeds The desired ChassisSpeeds.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setModuleStatesClosedLoop(DriveConstants.kinematics.toSwerveModuleStates(chassisSpeeds));
    }
    

    /**
     * Returns an array of module states of the drive base. The order is FL, FR, BL, BR.
     * 
     * @return An array of SwerveModuleState.
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            new SwerveModuleState(frontLeftMod.getVelocityMetersPerSec(), frontLeftMod.getSteerEncAngle()),
            new SwerveModuleState(frontRightMod.getVelocityMetersPerSec(), frontRightMod.getSteerEncAngle()),
            new SwerveModuleState(backRightMod.getVelocityMetersPerSec(), backRightMod.getSteerEncAngle()),
             new SwerveModuleState(backLeftMod.getVelocityMetersPerSec(), backLeftMod.getSteerEncAngle())
        };
    }

    /**
     * Returns an array of CANcoder angles of the modules. The order is FL, FR, BL, BR.
     * 
     * @return An array of Rotation2d.
     */
    public Rotation2d[] getCanCoderAngles() {
        return new Rotation2d[] {
            frontLeftMod.getCanCoderAngle(),
            frontRightMod.getCanCoderAngle(),
            backRightMod.getCanCoderAngle(),
            backLeftMod.getCanCoderAngle()
        };
    }

    /**
     * Returns an array of module positions.
     * 
     * @return An array of SwerveModulePosition.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftMod.getPosition(),
            frontRightMod.getPosition(),
            backRightMod.getPosition(),
            backLeftMod.getPosition()
        };
    }

    /**
     * @return The current estimated position of the robot on the field
     * based on drive encoder and gyro readings.
     */
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Resets the current pose to (0, 0) with a heading of zero.
     */
    public void resetPose() {
        resetDriveDistances();
        resetHeading();

        odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            imu.getRotation2d(),
            getModulePositions(),
            new Pose2d()
        );
    }

    public void setHeading(Rotation2d heading) {
        imu.setYaw(MathUtil.inputModulus(heading.getDegrees(), 0.0, 360.0));
    }

    /**
     * Sets the pose of the robot.
     * 
     * @param pose The pose to set the robot to.
     */
    public void setPose(Pose2d pose) {
        setHeading(pose.getRotation());

        odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            pose.getRotation(),
            getModulePositions(),
            pose
        );
    }

    public void setTranslation(Translation2d translation) {
        odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d(translation, getHeading())
        );
    }

    /**
     * Resets the measured distance driven for each module to zero.
     * <p>Resets the drive encoders of each module to zero.
     */
    public void resetDriveDistances() {
        frontLeftMod.resetDriveDistance();
        frontRightMod.resetDriveDistance();
        backRightMod.resetDriveDistance();
        backLeftMod.resetDriveDistance();
    }

    /**
     * Returns the average distance driven of each module to get an overall distance driven by the robot.
     * 
     * @return The overall distance driven by the robot in meters.
     */
    public double getAverageDriveDistanceMeters() {
        return (
            (frontLeftMod.getDriveDistanceMeters()
            + frontRightMod.getDriveDistanceMeters()
            + backRightMod.getDriveDistanceMeters()
            + backLeftMod.getDriveDistanceMeters())
            / 4.0
        );
    }

    /**
     * Returns the average velocity of each module to get an overall velocity of the robot.
     * 
     * @return The overall velocity of the robot in meters per second.
     */
    public double getAverageDriveVelocityMetersPerSec() {
        return (
            (Math.abs(frontLeftMod.getVelocityMetersPerSec())
            + Math.abs(frontRightMod.getVelocityMetersPerSec())
            + Math.abs(backRightMod.getVelocityMetersPerSec())
            + Math.abs(backLeftMod.getVelocityMetersPerSec()))
            / 4.0
        );
    }

    /**
     * Returns the average direction of each module to get an overall direction of travel of the robot.
     * 
     * @return The overall direction of travel of the robot.
     */
    public Rotation2d getDirectionOfTravel() {
        return new Rotation2d(
            (frontLeftMod.getSteerEncAngle().plus(new Rotation2d(frontLeftMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0)).getRadians()
            + frontRightMod.getSteerEncAngle().plus(new Rotation2d(frontRightMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0)).getRadians()
            + backRightMod.getSteerEncAngle().plus(new Rotation2d(backRightMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0)).getRadians()
            + backLeftMod.getSteerEncAngle().plus(new Rotation2d(backLeftMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0)).getRadians()
            ) / 4.0
        );
    }

    /**
     * Returns the average velocity in the direction relative to the robot.
     * 
     * @param relativeHeading The relative heading of the robot, where zero is the front of the robot.
     * 
     * @return The velocity in the direction relative to the robot in meters per second.
     */
    public double getRelativeVelocityMetersPerSec(Rotation2d relativeHeading) {
        return getDirectionOfTravel().minus(relativeHeading).getCos() * getAverageDriveVelocityMetersPerSec();
    }

    /**
     * Returns the current heading of the robot from the gyro.
     * 
     * @return The current heading of the robot as a Rotation2d.
     */
    public Rotation2d getHeading() {
        return imu.getRotation2d();
        //return Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(imu.getYaw().getValueAsDouble())));
    }

    /**
     * Returns the current pitch of the robot from the gyro.
     * 
     * @return The current pitch of the robot as a Rotation2d.
     */
    public Rotation2d getPitch() {
        // IMU is turned 90 degrees, so pitch and roll are flipped.
        return Rotation2d.fromDegrees(imu.getRoll().getValueAsDouble());
    }

    /**
     * Returns the current roll of the robot from the gyro.
     * 
     * @return The current roll of the robot as a Rotation2d.
     */
    public Rotation2d getRollDegrees() {
        // IMU is turned 90 degrees, so pitch and roll are flipped.
        return Rotation2d.fromDegrees(imu.getPitch().getValueAsDouble());
    }

    /**
     * Sets the gyro heading to zero.
     */
    public void resetHeading() {
        imu.setYaw(0.0);
    }

    /**
     * Sets the current limit of the drive motors of each module to the desired amperage.
     * 
     * @param amps The desired current limit of the drive motors in amps.
     */
    public void setDriveCurrentLimit(int amps) {
        frontLeftMod.setDriveCurrentLimit(amps);
        frontRightMod.setDriveCurrentLimit(amps);
        backRightMod.setDriveCurrentLimit(amps);
        backLeftMod.setDriveCurrentLimit(amps);
    }

    public void updateInterface() {
        SmartDashboard.putNumber("heading degrees", getHeading().getDegrees());
        SmartDashboard.putNumber("speed m/s", getAverageDriveVelocityMetersPerSec());
        SmartDashboard.putNumber("X Pos", getPose().getX());
        SmartDashboard.putNumber("Y Pos", getPose().getY());
        SmartDashboard.putBoolean("isFieldOriented", isFieldOriented);

        SmartDashboard.putNumber("FL angle degrees", getModuleStates()[0].angle.getDegrees());
        SmartDashboard.putNumber("FR angle degrees", getModuleStates()[1].angle.getDegrees());
        SmartDashboard.putNumber("BL angle degrees", getModuleStates()[2].angle.getDegrees());
        SmartDashboard.putNumber("BR angle degrees", getModuleStates()[3].angle.getDegrees());

        SmartDashboard.putNumber("FL raw CANCoder degrees", this.getCanCoderAngles()[0].getDegrees());
        SmartDashboard.putNumber("FR raw CANCoder degrees", this.getCanCoderAngles()[1].getDegrees());
        SmartDashboard.putNumber("BL raw CANCoder degrees", this.getCanCoderAngles()[2].getDegrees());
        SmartDashboard.putNumber("BR raw CANCoder degrees", this.getCanCoderAngles()[3].getDegrees());

        SmartDashboard.putNumber("FL offset CANCoder degrees", this.getCanCoderAngles()[0].getDegrees() - DriveConstants.frontLeftModOffset.getDegrees());
        SmartDashboard.putNumber("FR offset CANCoder degrees", this.getCanCoderAngles()[1].getDegrees() - DriveConstants.frontRightModOffset.getDegrees());
        SmartDashboard.putNumber("BL offset CANCoder degrees", this.getCanCoderAngles()[2].getDegrees() - DriveConstants.backLeftModOffset.getDegrees());
        SmartDashboard.putNumber("BR offset CANCoder degrees", this.getCanCoderAngles()[3].getDegrees() - DriveConstants.backRightModOffset.getDegrees());

        
        


    }
}