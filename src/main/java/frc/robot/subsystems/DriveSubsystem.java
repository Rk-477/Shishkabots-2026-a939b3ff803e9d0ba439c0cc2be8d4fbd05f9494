// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  // Commented out limelight camera as it's no longer used
    private LimelightSubsystem m_LimelightSubsystem;
    // PoseEstimator for tracking robot pose
    private PoseEstimator m_PoseEstimator;

    // Locations for the swerve drive modules relative to the robot center
    private final Translation2d m_frontLeftLocation = DriveConstants.FRONT_LEFT_LOCATION;
    private final Translation2d m_frontRightLocation = DriveConstants.FRONT_RIGHT_LOCATION;
    private final Translation2d m_backLeftLocation = DriveConstants.BACK_LEFT_LOCATION;
    private final Translation2d m_backRightLocation = DriveConstants.BACK_RIGHT_LOCATION;

    // Slew rate limiters to make joystick inputs more gentle
    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(DriveConstants.MAX_MAGNITUDE_SLEW_RATE);
    private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(DriveConstants.MAX_MAGNITUDE_SLEW_RATE);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.MAX_ROTATIONAL_SLEW_RATE_RPS);

    // Motor controllers for the swerve drive modules
    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.DRIVE_FRONT_LEFT_CAN_ID, 
        DriveConstants.DRIVE_TURN_FRONT_LEFT_CAN_ID, 
        DriveConstants.FRONT_LEFT_CHASIS_ANGULAR_OFFSET,
        true, "FrontLeft");

    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.DRIVE_FRONT_RIGHT_CAN_ID, 
        DriveConstants.DRIVE_TURN_FRONT_RIGHT_CAN_ID, 
        DriveConstants.FRONT_RIGHT_CHASIS_ANGULAR_OFFSET,
        false, "FrontRight");
        
    private final SwerveModule m_backLeft = new SwerveModule(
        DriveConstants.DRIVE_REAR_LEFT_CAN_ID, 
        DriveConstants.DRIVE_TURN_REAR_LEFT_CAN_ID, 
        DriveConstants.BACK_LEFT_CHASIS_ANGULAR_OFFSET,
        true, "BackLeft");

    private final SwerveModule m_backRight = new SwerveModule(
        DriveConstants.DRIVE_REAR_RIGHT_CAN_ID, 
        DriveConstants.DRIVE_TURN_REAR_RIGHT_CAN_ID, 
        DriveConstants.BACK_RIGHT_CHASIS_ANGULAR_OFFSET,
        false, "BackRight");

    // Use DriveConstants.MAX_SPEED_IN_MPS for consistency
    
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.PIGEON_CAN_ID); // Update the ID based on your Pigeon's CAN ID
    // initialize the field for simulator tracking
    private final Field2d m_field = new Field2d();

    private int updateCounter = 0;

    private DoubleLogEntry m_speedLog;
    private DoubleLogEntry m_headingLog;

  public DriveSubsystem(LimelightSubsystem limelight) {
    this.m_LimelightSubsystem = limelight;
    m_PoseEstimator = new PoseEstimator(this, m_LimelightSubsystem);
        // Reset the gyro
        m_gyro.reset();

        // log field into smartdashboard
        SmartDashboard.putData("Field", m_field);

        try {
            DriveConstants.pathPlannerConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // PathPlanner settings not found - create a default config for simulation
            System.err.println("PathPlanner settings.json not found, using default config: " + e.getMessage());
            try {
                DriveConstants.pathPlannerConfig = new RobotConfig(
                    50.0, // mass in kg (estimated)
                    6.0,  // MOI (moment of inertia)
                    new com.pathplanner.lib.config.ModuleConfig(
                        0.0508, // wheel radius in meters (2 inches)
                        4.0,    // max drive velocity m/s
                        1.0,    // wheel COF
                        DCMotor.getNEO(1).withReduction(6.75), // drive motor with gear ratio
                        60.0,   // drive current limit
                        1       // number of drive motors per module
                    ),
                    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
                );
            } catch (Exception ex) {
                System.err.println("Failed to create default RobotConfig: " + ex.getMessage());
            }
        }

        // Skip AutoBuilder if config failed to load
        if (DriveConstants.pathPlannerConfig == null) {
            System.err.println("WARNING: PathPlanner AutoBuilder not configured - no valid RobotConfig");
        } else {
            // Configure AutoBuilder
            AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
                ),
                DriveConstants.pathPlannerConfig, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false; 
                }, 
                this // Reference to this subsystem to set requirements
        );
        }

        // Initialize DataLogManager entries
        DataLog log = DataLogManager.getLog();
        m_speedLog = new DoubleLogEntry(log, "/drive/speed");
        m_headingLog = new DoubleLogEntry(log, "/drive/heading");
  }

   /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     */
    public void drive(double xSpeed, double ySpeed, double rot) {
        drive(xSpeed, ySpeed, rot, true); // Default to field-relative
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether to use field-relative driving.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // If all inputs are zero, stop the motors
        if (Math.abs(xSpeed) < 1E-6 && Math.abs(ySpeed) < 1E-6 && Math.abs(rot) < 1E-6) {
            holdModules();
            return;
        }

        // Smooth command changes so driving feels controlled.
        xSpeed = m_xSpeedLimiter.calculate(xSpeed);
        ySpeed = m_ySpeedLimiter.calculate(ySpeed);
        rot = m_rotLimiter.calculate(rot);

        // Convert the commanded speeds from [-1, 1] to real speeds
        xSpeed = xSpeed * DriveConstants.MAX_SPEED_IN_MPS;
        ySpeed = ySpeed * DriveConstants.MAX_SPEED_IN_MPS;
        rot = rot * DriveConstants.MAX_ANGULAR_SPEED_IN_RPS;

        // Create chassis speeds - field relative or robot relative
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroRotation());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        // Calculate and apply module states
        var swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_IN_MPS);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }
    /**
     * Drive using ChassisSpeeds (already in m/s and rad/s units).
     * Used by PathPlanner and autonomous commands.
     */
    public void drive(ChassisSpeeds speeds) {
        // ChassisSpeeds are already in real units (m/s, rad/s), not [-1, 1]
        // So we need to set module states directly without scaling
        var swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_IN_MPS);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
        m_xSpeedLimiter.reset(0.0);
        m_ySpeedLimiter.reset(0.0);
        m_rotLimiter.reset(0.0);
    }

    /**
     * Hold module steering angles and zero drive speed to resist passive wheel rotation.
     */
    public void holdModules() {
        m_frontLeft.holdPosition();
        m_frontRight.holdPosition();
        m_backLeft.holdPosition();
        m_backRight.holdPosition();
        m_xSpeedLimiter.reset(0.0);
        m_ySpeedLimiter.reset(0.0);
        m_rotLimiter.reset(0.0);
    }

    /**
     * Returns the gyro rotation as a Rotation2d object
     */
    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
    }

    /**
     * Returns the current pose of the robot
     */
    public Pose2d getPose() {
        return m_PoseEstimator.getPose2d();
    }

    /**
     * Resets the odometry to a known pose
     */
    public void resetOdometry(Pose2d pose) {
        m_PoseEstimator.setCurrentPose(pose);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveDriveKinematics getDriveKinematics() {
        return kinematics;
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     // Only update logging every 50 cycles to reduce processing load and network traffic
        updateCounter++;
        if (updateCounter >= 10) {
            try {
                // set robot position in the field (keep this for field visualization)
                m_field.setRobotPose(m_PoseEstimator.getPose2d());
                
                // log array of all swerve modules to be put into advantagescope simulation
                double loggingState[] = {
                    m_frontLeft.getSteerAngle(),
                    m_frontLeft.getDriveSpeed(),
                    m_frontRight.getSteerAngle(),
                    m_frontRight.getDriveSpeed(),
                    m_backLeft.getSteerAngle(),
                    m_backLeft.getDriveSpeed(),
                    m_backRight.getSteerAngle(),
                    m_backRight.getDriveSpeed()
                };

                // Keep this for visualization tools but remove other SmartDashboard updates
                SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);

                // === CALIBRATION OUTPUT ===
                // Point all wheels forward manually, then read these values (in radians)
                // Copy these values to Constants.java as angular offsets
                SmartDashboard.putNumber("Calibration/FrontLeft", m_frontLeft.getRawAbsoluteEncoderPosition());
                SmartDashboard.putNumber("Calibration/FrontRight", m_frontRight.getRawAbsoluteEncoderPosition());
                SmartDashboard.putNumber("Calibration/BackLeft", m_backLeft.getRawAbsoluteEncoderPosition());
                SmartDashboard.putNumber("Calibration/BackRight", m_backRight.getRawAbsoluteEncoderPosition());

                // Show calibration in DEGREES for easier reading
                SmartDashboard.putNumber("Calibration/FL_Degrees", Math.toDegrees(m_frontLeft.getRawAbsoluteEncoderPosition()));
                SmartDashboard.putNumber("Calibration/FR_Degrees", Math.toDegrees(m_frontRight.getRawAbsoluteEncoderPosition()));
                SmartDashboard.putNumber("Calibration/BL_Degrees", Math.toDegrees(m_backLeft.getRawAbsoluteEncoderPosition()));
                SmartDashboard.putNumber("Calibration/BR_Degrees", Math.toDegrees(m_backRight.getRawAbsoluteEncoderPosition()));

                // Show gyro heading
                SmartDashboard.putNumber("Drive/Gyro_Degrees", getGyroRotation().getDegrees());
                SmartDashboard.putNumber("Drive/Gyro_Radians", getGyroRotation().getRadians());

                // --- Per-module debug values for Shuffleboard ---
                SmartDashboard.putNumber("Drive/FrontLeft/MeasuredAngleRad", m_frontLeft.getSteerAngle());
                SmartDashboard.putNumber("Drive/FrontRight/MeasuredAngleRad", m_frontRight.getSteerAngle());
                SmartDashboard.putNumber("Drive/BackLeft/MeasuredAngleRad", m_backLeft.getSteerAngle());
                SmartDashboard.putNumber("Drive/BackRight/MeasuredAngleRad", m_backRight.getSteerAngle());

                SmartDashboard.putNumber("Drive/FrontLeft/MeasuredAngleDeg", Math.toDegrees(m_frontLeft.getSteerAngle()));
                SmartDashboard.putNumber("Drive/FrontRight/MeasuredAngleDeg", Math.toDegrees(m_frontRight.getSteerAngle()));
                SmartDashboard.putNumber("Drive/BackLeft/MeasuredAngleDeg", Math.toDegrees(m_backLeft.getSteerAngle()));
                SmartDashboard.putNumber("Drive/BackRight/MeasuredAngleDeg", Math.toDegrees(m_backRight.getSteerAngle()));

                SmartDashboard.putNumber("Drive/FrontLeft/DesiredAngleRad", m_frontLeft.getDesiredAngle());
                SmartDashboard.putNumber("Drive/FrontRight/DesiredAngleRad", m_frontRight.getDesiredAngle());
                SmartDashboard.putNumber("Drive/BackLeft/DesiredAngleRad", m_backLeft.getDesiredAngle());
                SmartDashboard.putNumber("Drive/BackRight/DesiredAngleRad", m_backRight.getDesiredAngle());

                SmartDashboard.putNumber("Drive/FrontLeft/DriveSpeedMps", m_frontLeft.getDriveSpeed());
                SmartDashboard.putNumber("Drive/FrontRight/DriveSpeedMps", m_frontRight.getDriveSpeed());
                SmartDashboard.putNumber("Drive/BackLeft/DriveSpeedMps", m_backLeft.getDriveSpeed());
                SmartDashboard.putNumber("Drive/BackRight/DriveSpeedMps", m_backRight.getDriveSpeed());

                SmartDashboard.putNumber("Drive/FrontLeft/DesiredSpeedMps", m_frontLeft.getDesiredSpeed());
                SmartDashboard.putNumber("Drive/FrontRight/DesiredSpeedMps", m_frontRight.getDesiredSpeed());
                SmartDashboard.putNumber("Drive/BackLeft/DesiredSpeedMps", m_backLeft.getDesiredSpeed());
                SmartDashboard.putNumber("Drive/BackRight/DesiredSpeedMps", m_backRight.getDesiredSpeed());

                SmartDashboard.putNumber("Drive/FrontLeft/DriveVoltage", m_frontLeft.getDriveVoltage());
                SmartDashboard.putNumber("Drive/FrontRight/DriveVoltage", m_frontRight.getDriveVoltage());
                SmartDashboard.putNumber("Drive/BackLeft/DriveVoltage", m_backLeft.getDriveVoltage());
                SmartDashboard.putNumber("Drive/BackRight/DriveVoltage", m_backRight.getDriveVoltage());

                SmartDashboard.putNumber("Drive/FrontLeft/TurnVoltage", m_frontLeft.getTurningVoltage());
                SmartDashboard.putNumber("Drive/FrontRight/TurnVoltage", m_frontRight.getTurningVoltage());
                SmartDashboard.putNumber("Drive/BackLeft/TurnVoltage", m_backLeft.getTurningVoltage());
                SmartDashboard.putNumber("Drive/BackRight/TurnVoltage", m_backRight.getTurningVoltage());
            } catch (Exception e) {
                System.err.println("Error updating dashboard: " + e.getMessage());
            }
            updateCounter = 0;
        }
  }
  public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_IN_MPS);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }

    public Rotation2d getHeading() {
        return getGyroRotation();
    }

    /**
     * Resets the gyro to zero. Call this when the robot is facing "forward"
     * to set the field-relative forward direction.
     */
    public void zeroHeading() {
        m_gyro.reset();
    }
    
    public Command driveToEndPose(Pose2d endPose) {
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

        return AutoBuilder.pathfindToPose(endPose, constraints, 0.0);
    }
    @Override
    public void simulationPeriodic() {
        m_frontLeft.updateSimulatorState();
        m_frontRight.updateSimulatorState();
        m_backLeft.updateSimulatorState();
        m_backRight.updateSimulatorState();

        double angularVelocity = kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond;
        updateGyroSimulatorState(angularVelocity);
    }

    public void updateGyroSimulatorState(double angularVelocity) {
        // convert radians per second to degrees per second
        double angularVelocityDegrees = angularVelocity * (180 / Math.PI);
        double newYaw = m_gyro.getYaw().getValueAsDouble() +  angularVelocityDegrees * 0.02;
        m_gyro.getSimState().setRawYaw(newYaw);
    }

    // ==================== WHEEL TEST METHODS ====================
    // Use these to test individual motors for debugging

    /**
     * Set all drive motors to the same power (for testing wheels spin)
     * @param power Power from -1.0 to 1.0
     */
    public void testDriveMotors(double power) {
        m_frontLeft.setDrivePower(power);
        m_frontRight.setDrivePower(power);
        m_backLeft.setDrivePower(power);
        m_backRight.setDrivePower(power);
    }

    /**
     * Set all turning motors to the same power (for testing swivel)
     * @param power Power from -1.0 to 1.0
     */
    public void testTurningMotors(double power) {
        m_frontLeft.setTurningPower(power);
        m_frontRight.setTurningPower(power);
        m_backLeft.setTurningPower(power);
        m_backRight.setTurningPower(power);
    }

    /**
     * Set all wheels to point at a specific angle (for testing swivel PID)
     * @param angleDegrees Target angle in degrees (0 = forward)
     */
    public void setAllWheelAngles(double angleDegrees) {
        double angleRadians = Math.toRadians(angleDegrees);
        m_frontLeft.setTurningAngle(angleRadians);
        m_frontRight.setTurningAngle(angleRadians);
        m_backLeft.setTurningAngle(angleRadians);
        m_backRight.setTurningAngle(angleRadians);
    }
}
