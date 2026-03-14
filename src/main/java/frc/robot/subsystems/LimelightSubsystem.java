package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

import java.util.function.Supplier;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable m_limelightTable;
    private AprilTagFieldLayout aprilTagField = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private double millisTimeRecorded;

    private double yaw;

    // NetworkTable entries for common Limelight values
    private NetworkTableEntry tx;  // Horizontal offset from crosshair to target
    private NetworkTableEntry ty;  // Vertical offset from crosshair to target
    private NetworkTableEntry ta;  // Target area (0% to 100% of image)
    private NetworkTableEntry tv;  // Whether the limelight has any valid targets (0 or 1)

    // Field2d for AdvantageScope visualization
    private final Field2d m_visionField = new Field2d();

    // Struct publishers for AdvantageScope 3D visualization
    private final StructPublisher<Pose3d> m_visionPose3dPublisher;
    private final StructPublisher<Pose3d> m_tagPosePublisher;

    // Simulation support
    private Supplier<Pose2d> m_robotPoseSupplier = null;
    private int m_simTagId = 0;
    private double m_simTx = 0;
    private double m_simTy = 0;
    private boolean m_simHasTarget = false;
    private static final double SIM_MAX_DISTANCE = 5.0; // Max distance to detect tags in sim (meters)
    private static final double SIM_FOV_HORIZONTAL = 27.0; // Limelight FOV in degrees
    
    public LimelightSubsystem() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        // Initialize NetworkTable entries
        tx = m_limelightTable.getEntry("tx");
        ty = m_limelightTable.getEntry("ty");
        ta = m_limelightTable.getEntry("ta");
        tv = m_limelightTable.getEntry("tv");

        // Initialize AdvantageScope publishers for 3D visualization
        NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        m_visionPose3dPublisher = visionTable.getStructTopic("VisionPose3d", Pose3d.struct).publish();
        m_tagPosePublisher = visionTable.getStructTopic("DetectedTagPose", Pose3d.struct).publish();

        // Publish vision field for 2D visualization
        SmartDashboard.putData("VisionField", m_visionField);

        // Set default pipeline
        setPipeline(0);
    }
    
    @Override
    public void periodic() {
        // Update values from NetworkTables
        double currentX = tx.getDouble(0.0);
        double currentY = ty.getDouble(0.0);
        double currentArea = ta.getDouble(0.0);
        double currentTarget = tv.getDouble(0.0);

        millisTimeRecorded = WPIUtilJNI.now() * 1e-3;

        // You can also log these values to SmartDashboard for debugging
        SmartDashboard.putNumber("Limelight X", currentX);
        SmartDashboard.putNumber("Limelight Y", currentY);
        SmartDashboard.putNumber("Limelight Area", currentArea);
        SmartDashboard.putBoolean("Limelight Has Target", currentTarget > 0.5);
        SmartDashboard.putNumber("Limelight distance", getDistanceFromTag(1.6, -getX()));
        SmartDashboard.putNumber("Limelight Tag ID", getTargetID());

        Pose3d botPose = getBotPose3d();
        if (botPose != null) {
            Rotation3d rotation = botPose.getRotation();
            SmartDashboard.putBoolean("Limelight Botpose Valid", true);
            SmartDashboard.putNumber("Limelight Roll", Math.toDegrees(rotation.getX()));
            SmartDashboard.putNumber("Limelight Pitch", Math.toDegrees(rotation.getY()));
            SmartDashboard.putNumber("Limelight Yaw", Math.toDegrees(rotation.getZ()));
        } else {
            SmartDashboard.putBoolean("Limelight Botpose Valid", false);
            SmartDashboard.putNumber("Limelight Roll", Double.NaN);
            SmartDashboard.putNumber("Limelight Pitch", Double.NaN);
            SmartDashboard.putNumber("Limelight Yaw", Double.NaN);
        }

        // Publish detected AprilTag pose to AdvantageScope for 3D field visualization
        if (isTargetValid()) {
            int tagId = getTargetID();
            Pose3d tagPose = aprilTagField.getTagPose(tagId).orElse(null);
            if (tagPose != null) {
                m_tagPosePublisher.set(tagPose);
                SmartDashboard.putNumber("Detected Tag ID", tagId);
            }
        }
    }
    
    
    /**
     * @return horizontal offset from crosshair to target (-27 degrees to 27 degrees)
     */
    public double getX() {
        return tx.getDouble(0.0);
    }
    
    /**
     * @return vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees)
     */
    public double getY() {
        return ty.getDouble(0.0);
    }
    
    /**
     * @return target area (0% to 100% of image)
     */
    public double getArea() {
        return ta.getDouble(0.0);
    }
    
    /**
     * @return whether the limelight has any valid targets (0 or 1)
     */
    public boolean hasValidTarget() {
        double targetValue = getRawTargetValue();
        return targetValue > 0.5;
    }
    
    /**
     * Sets LED mode
     * @param mode 0 = use pipeline mode, 1 = force off, 2 = force blink, 3 = force on
     */
    public void setLEDMode(int mode) {
        m_limelightTable.getEntry("ledMode").setNumber(mode);
    }
    
    /**
     * Sets camera mode
     * @param mode 0 = vision processor, 1 = driver camera
     */
    public void setCameraMode(int mode) {
        m_limelightTable.getEntry("camMode").setNumber(mode);
    }
    
    /**
     * Sets current pipeline
     * @param pipeline Pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        m_limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
    
    /**
     * Sets the exposure time for the camera
     * @param exposure Exposure time in milliseconds (0-100)
     */
    public void setExposure(double exposure) {
        m_limelightTable.getEntry("exposure").setNumber(exposure);
    }

    /**
     * Sets the black level offset
     * @param blackLevel Black level offset (0-100)
     */
    public void setBlackLevel(double blackLevel) {
        m_limelightTable.getEntry("black_level").setNumber(blackLevel);
    }

    /**
     * Gets the raw target detection value from NetworkTables
     * @return raw tv value (0.0 if no target, 1.0 if target detected)
     */
    public double getRawTargetValue() {
        return m_limelightTable.getEntry("tv").getDouble(0.0);
    }

    /**
     * Gets the detected AprilTag ID
     * @return AprilTag ID number, or 0.0 if no tag detected
     */
    public int getTargetID() {
        return (int) m_limelightTable.getEntry("tid").getDouble(0.0);
    }

    /**
     * Gets the field-layout height (Z) of a tag by ID.
     * @param tagId AprilTag ID
     * @return tag height in meters, or NaN if the ID does not exist in layout
     */
    public double getTagHeightMeters(int tagId) {
        if (tagId <= 0) {
            return Double.NaN;
        }

        Pose3d tagPose = aprilTagField.getTagPose(tagId).orElse(null);
        if (tagPose == null) {
            return Double.NaN;
        }

        return tagPose.getZ();
    }

    public boolean isTargetValid() {
        return m_limelightTable.getEntry("tv").getDouble(0.0) == 1;
    }
    
    // get the timestamp of robot in miliseconds
    public double getTimeRecordedInMilis() {
        return millisTimeRecorded;
    }

    public double getDistanceFromTag(double tagHeight, double tagYDiff) {
        double heightDiff = tagHeight - LimelightConstants.kCameraToRobot.getZ();
        return heightDiff / Math.tan(Math.toRadians(-getX()));
    }
    /*
     * If it detects an AprilTag ID, get it's pose to estimate the robot location with tx, ty, heightDiff
     */
    public double getYaw() {
        return yaw;
    }
    public synchronized Pose2d getPose(Rotation2d robotRotation2d) {
        // if not valid return null
        if (!isTargetValid()) {
            return null;
        }
        Pose3d tag = aprilTagField.getTagPose(getTargetID()).orElse(null);
        if (tag == null) {
            return null;
        }
        double yaw = robotRotation2d.getRadians();
        double distance = getDistanceFromTag(tag.getZ(), -getX());
        double beta = yaw - Math.toRadians(getY());
        double x = Math.cos(beta) * distance;
        double y = Math.sin(beta) * distance;
        Translation2d tagToCamera = new Translation2d(-x, -y);

        Pose2d cameraPose =
        new Pose2d(tag.toPose2d().getTranslation().plus(tagToCamera), new Rotation2d(yaw));

        Translation2d offset = LimelightConstants.kCameraToRobot.toTranslation2d().rotateBy(robotRotation2d);
        Pose2d robotPose = new Pose2d(cameraPose.getTranslation().minus(offset), new Rotation2d(yaw));

        // Publish to AdvantageScope for visualization
        m_visionField.setRobotPose(robotPose);
        m_visionPose3dPublisher.set(new Pose3d(robotPose));

        return robotPose;
    }

    /**
     * Gets the robot pose directly from Limelight's MegaTag/botpose
     * This uses Limelight's built-in pose estimation (botpose_wpiblue)
     * @return Pose3d of the robot, or null if no valid pose
     */
    public Pose3d getBotPose3d() {
        double[] botpose = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        if (botpose.length < 6) {
            return null;
        }

        Pose3d pose = new Pose3d(
            botpose[0],  // x
            botpose[1],  // y
            botpose[2],  // z
            new Rotation3d(
                Math.toRadians(botpose[3]),  // roll
                Math.toRadians(botpose[4]),  // pitch
                Math.toRadians(botpose[5])   // yaw
            )
        );

        // Publish to AdvantageScope
        m_visionPose3dPublisher.set(pose);
        m_visionField.setRobotPose(pose.toPose2d());

        return pose;
    }

    /**
     * Updates the vision pose for AdvantageScope - call this from robot periodic
     * @param robotRotation Current robot rotation from gyro
     */
    public void updateVisionPose(Rotation2d robotRotation) {
        // Try MegaTag pose first (more accurate)
        Pose3d botPose = getBotPose3d();
        if (botPose != null) {
            return; // Already published in getBotPose3d
        }

        // Fall back to calculated pose
        getPose(robotRotation);
    }

    /**
     * Sets the robot pose supplier for simulation mode
     * @param poseSupplier A supplier that returns the current robot pose
     */
    public void setRobotPoseSupplier(Supplier<Pose2d> poseSupplier) {
        m_robotPoseSupplier = poseSupplier;
    }

    @Override
    public void simulationPeriodic() {
        if (m_robotPoseSupplier == null) {
            return;
        }

        Pose2d robotPose = m_robotPoseSupplier.get();
        if (robotPose == null) {
            return;
        }

        // Find the closest visible AprilTag
        m_simHasTarget = false;
        m_simTagId = 0;
        double closestDistance = Double.MAX_VALUE;
        Pose3d closestTag = null;

        for (int tagId = 1; tagId <= 22; tagId++) { // Check all possible tag IDs
            var tagPoseOpt = aprilTagField.getTagPose(tagId);
            if (tagPoseOpt.isEmpty()) continue;

            Pose3d tagPose = tagPoseOpt.get();
            Pose2d tagPose2d = tagPose.toPose2d();

            // Calculate distance to tag
            double distance = robotPose.getTranslation().getDistance(tagPose2d.getTranslation());

            // Check if tag is within range
            if (distance > SIM_MAX_DISTANCE) continue;

            // Calculate angle to tag from robot's perspective
            Translation2d robotToTag = tagPose2d.getTranslation().minus(robotPose.getTranslation());
            double angleToTag = Math.toDegrees(Math.atan2(robotToTag.getY(), robotToTag.getX()));
            double robotHeading = robotPose.getRotation().getDegrees();
            double relativeAngle = angleToTag - robotHeading;

            // Normalize angle to -180 to 180
            while (relativeAngle > 180) relativeAngle -= 360;
            while (relativeAngle < -180) relativeAngle += 360;

            // Check if tag is within FOV
            if (Math.abs(relativeAngle) > SIM_FOV_HORIZONTAL) continue;

            // This tag is visible - check if it's the closest
            if (distance < closestDistance) {
                closestDistance = distance;
                m_simTagId = tagId;
                m_simTx = relativeAngle;
                m_simTy = Math.toDegrees(Math.atan2(tagPose.getZ() - LimelightConstants.kCameraToRobot.getZ(), distance));
                m_simHasTarget = true;
                closestTag = tagPose;
            }
        }

        // Publish simulated values to NetworkTables (so the rest of the code works)
        if (m_simHasTarget) {
            m_limelightTable.getEntry("tv").setDouble(1.0);
            m_limelightTable.getEntry("tid").setDouble(m_simTagId);
            m_limelightTable.getEntry("tx").setDouble(m_simTx);
            m_limelightTable.getEntry("ty").setDouble(m_simTy);
            m_limelightTable.getEntry("ta").setDouble(100.0 / (closestDistance * closestDistance)); // Fake area based on distance

            // Publish simulated botpose_wpiblue
            double[] botpose = new double[] {
                robotPose.getX(),
                robotPose.getY(),
                0.0, // z
                0.0, // roll
                0.0, // pitch
                robotPose.getRotation().getDegrees() // yaw
            };
            m_limelightTable.getEntry("botpose_wpiblue").setDoubleArray(botpose);

            // Publish vision pose to AdvantageScope
            Pose3d visionPose = new Pose3d(robotPose);
            m_visionPose3dPublisher.set(visionPose);
            m_visionField.setRobotPose(robotPose);

            if (closestTag != null) {
                m_tagPosePublisher.set(closestTag);
            }
        } else {
            m_limelightTable.getEntry("tv").setDouble(0.0);
            m_limelightTable.getEntry("tid").setDouble(0.0);
        }
    }
}
