package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import util.Logger;

public class AutoAllign extends Command {
  private final LimelightSubsystem limelight;
  private final DriveSubsystem drive;

  // PID controllers for alignment.
  private final PIDController distancePID = new PIDController(0.8, 0, 0.05);
  private final PIDController strafePID = new PIDController(0.05, 0, 0.005);
  private final PIDController rotationPID = new PIDController(0.03, 0, 0.002);

  // Fallback target distance if no per-tag override exists.
  private final double defaultTargetDistance;

  // Tolerances
  private static final double DISTANCE_TOLERANCE = 0.1;  // +-10 cm
  private static final double STRAFE_TOLERANCE = 2.0;    // +-2 degrees
  private static final double ROTATION_TOLERANCE = 3.0;  // +-3 degrees

  // Speed limits
  private static final double MAX_DRIVE_SPEED = 1.5;     // m/s
  private static final double MAX_ROTATION_SPEED = 2.0;  // rad/s

  /**
   * Creates a new AutoAlign command with per-tag target distance and default fallback.
   */
  public AutoAllign(LimelightSubsystem limelight, DriveSubsystem drive) {
    this(limelight, drive, AutoAlignConstants.DEFAULT_TARGET_DISTANCE_METERS);
  }

  /**
   * Creates a new AutoAlign command.
   *
   * @param limelight The limelight subsystem
   * @param drive The drive subsystem
   * @param defaultTargetDistanceMeters Fallback distance (meters) when no per-tag override exists
   */
  public AutoAllign(LimelightSubsystem limelight, DriveSubsystem drive, double defaultTargetDistanceMeters) {
    this.limelight = limelight;
    this.drive = drive;
    this.defaultTargetDistance = defaultTargetDistanceMeters;

    distancePID.setTolerance(DISTANCE_TOLERANCE);
    strafePID.setTolerance(STRAFE_TOLERANCE);
    rotationPID.setTolerance(ROTATION_TOLERANCE);

    addRequirements(limelight, drive);
  }

  @Override
  public void initialize() {
    distancePID.reset();
    strafePID.reset();
    rotationPID.reset();
    Logger.log("AutoAlign started - default target distance: " + defaultTargetDistance + "m");
  }

  @Override
  public void execute() {
    if (!limelight.hasValidTarget()) {
      drive.stop();
      Logger.log("AutoAlign: No valid target detected");
      return;
    }

    int tagId = limelight.getTargetID();
    double tx = limelight.getX();
    double ty = limelight.getY();

    double tagHeightMeters = limelight.getTagHeightMeters(tagId);
    if (Double.isNaN(tagHeightMeters)) {
      tagHeightMeters = AutoAlignConstants.DEFAULT_TAG_HEIGHT_METERS;
    }

    double targetDistance = getTargetDistanceForTag(tagId);
    double currentDistance = calculateDistanceFromTag(ty, tagHeightMeters, targetDistance);

    double distanceError = currentDistance - targetDistance;
    double strafeError = tx;
    double rotationError = tx;

    double vx = -distancePID.calculate(distanceError, 0);
    double vy = -strafePID.calculate(strafeError, 0);
    double omega = -rotationPID.calculate(rotationError, 0);

    vx = MathUtil.clamp(vx, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    vy = MathUtil.clamp(vy, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    omega = MathUtil.clamp(omega, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

    Logger.log(String.format(
        "AutoAlign: tag=%d, tagZ=%.2f, target=%.2f, dist=%.2f, err=%.2f, tx=%.1f, vx=%.2f, vy=%.2f, omega=%.2f",
        tagId, tagHeightMeters, targetDistance, currentDistance, distanceError, tx, vx, vy, omega));

    drive.drive(vx, vy, omega);
  }

  /**
   * Calculate distance from the AprilTag using vertical angle.
   *
   * @param ty Vertical angle to target in degrees
   * @param tagHeightMeters Height of the detected tag in meters
   * @param fallbackDistanceMeters Distance to return if angle is too small
   * @return Distance in meters
   */
  private double calculateDistanceFromTag(double ty, double tagHeightMeters, double fallbackDistanceMeters) {
    double cameraAngleDegrees = 0.0;  // TODO: Set your camera mount angle
    double cameraHeightMeters = AutoAlignConstants.CAMERA_HEIGHT_METERS;

    double totalAngleRadians = Math.toRadians(cameraAngleDegrees + ty);
    if (Math.abs(totalAngleRadians) < 0.01) {
      return fallbackDistanceMeters;
    }

    double distance = (tagHeightMeters - cameraHeightMeters) / Math.tan(totalAngleRadians);
    return Math.abs(distance);
  }

  private double getTargetDistanceForTag(int tagId) {
    return AutoAlignConstants.TARGET_DISTANCE_METERS_BY_TAG.getOrDefault(tagId, defaultTargetDistance);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.log("AutoAlign ended - interrupted: " + interrupted);
  }

  @Override
  public boolean isFinished() {
    if (!limelight.hasValidTarget()) {
      return false;
    }

    boolean aligned = distancePID.atSetpoint()
        && strafePID.atSetpoint()
        && rotationPID.atSetpoint();

    if (aligned) {
      Logger.log("AutoAlign: Alignment complete!");
    }

    return aligned;
  }
}
