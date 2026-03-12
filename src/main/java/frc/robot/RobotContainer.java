// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAllign;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(limelightSubsystem);
  // Shooter: Left CAN ID, Right CAN ID | Tower CAN ID | Conveyor CAN ID | Intake CAN ID
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(11, 9, 12, 13, 16);

  // The driver's controllers
  // Primary controller (port 0) is for the main driver
  // Secondary controller (port 1) is for the operator/co-pilot
  // Both controllers have the same button mappings for redundancy
  private final XboxController driveController = new XboxController(DRIVER_CONTROLLER_PORT); // Primary controller
  private final XboxController mechanismController = new XboxController(OPERATOR_CONTROLLER_PORT); // Secondary controller

  private static final double DEADBAND = 0.10;
  private static final double ROTATION_MULTIPLIER = 0.70;
  private static final double NORMAL_TRANSLATION_SCALE = 0.70;
  private static final double NORMAL_ROTATION_SCALE = 0.65;
  private static final double SPRINT_TRANSLATION_SCALE = 1.00;
  private static final double SPRINT_ROTATION_SCALE = 0.90;
  private static final boolean TELEOP_FIELD_RELATIVE = true;

  // setup the AutoBuilder with all pathplanner paths in place
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public LimelightSubsystem getLimelightSubsystem() {
    return limelightSubsystem;
  }

  private double applyDeadbandAndCurve(double value) {
    if (Math.abs(value) < DEADBAND) {
      return 0.0;
    }

    double sign = Math.signum(value);
    double adjusted = (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
    // Blend linear and cubic to keep fine control near center without feeling sluggish at mid-stick.
    double curved = 0.5 * adjusted + 0.5 * adjusted * adjusted * adjusted;
    return sign * curved;
  }

  private double[] getTranslationInputs() {
    XboxController controller = getActiveController();
    double rawForward = -controller.getLeftY();
    double rawStrafe = -controller.getLeftX();

    double magnitude = Math.hypot(rawForward, rawStrafe);
    if (magnitude < DEADBAND) {
      return new double[] {0.0, 0.0};
    }

    double adjustedMagnitude = (magnitude - DEADBAND) / (1.0 - DEADBAND);
    double curvedMagnitude =
        0.5 * adjustedMagnitude + 0.5 * adjustedMagnitude * adjustedMagnitude * adjustedMagnitude;

    double unitForward = rawForward / magnitude;
    double unitStrafe = rawStrafe / magnitude;
    return new double[] {unitForward * curvedMagnitude, unitStrafe * curvedMagnitude};
  }

  private double getTranslationScale() {
    return getActiveController().getRightBumperButton()
        ? SPRINT_TRANSLATION_SCALE
        : NORMAL_TRANSLATION_SCALE;
  }

  private double getRotationScale() {
    return getActiveController().getRightBumperButton() ? SPRINT_ROTATION_SCALE : NORMAL_ROTATION_SCALE;
  }

  private double getForwardInput() {
    return getTranslationInputs()[0];
  }

  private double getStrafeInput() {
    return getTranslationInputs()[1];
  }

  private double getTurnInput() {
    double rawTurn = -getActiveController().getRightX();
    double deadbanded = MathUtil.applyDeadband(rawTurn, DEADBAND);
    double cubic = deadbanded * deadbanded * deadbanded;
    return MathUtil.clamp(cubic * ROTATION_MULTIPLIER * getRotationScale(), -1.0, 1.0);
  }

  private XboxController getActiveController() {
    if (DriverStation.isJoystickConnected(DRIVER_CONTROLLER_PORT)) {
      return driveController;
    }
    if (DriverStation.isJoystickConnected(OPERATOR_CONTROLLER_PORT)) {
      return mechanismController;
    }
    return driveController;
  }

  private Trigger button(int buttonId) {
    return new Trigger(
        () -> driveController.getRawButton(buttonId) || mechanismController.getRawButton(buttonId));
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
     // Set up the default command for the drive subsystem (60% speed default)
    driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            driveSubsystem,
            () -> getForwardInput() * getTranslationScale(),
            () -> getStrafeInput() * getTranslationScale(),
            () -> getTurnInput(),
            TELEOP_FIELD_RELATIVE
        )
    );

    // Connect Limelight to robot pose for simulation
    limelightSubsystem.setRobotPoseSupplier(() -> driveSubsystem.getPose());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Press A: start vision auto-alignment (no hold required).
    button(XboxController.Button.kA.value)
        .onTrue(new AutoAllign(limelightSubsystem, driveSubsystem));

    // Start: zero gyro heading for drivebase.
    button(XboxController.Button.kStart.value)
        .onTrue(Commands.runOnce(driveSubsystem::zeroHeading, driveSubsystem));

    // Y / Back: point all wheels forward for quick alignment reset.
    button(XboxController.Button.kY.value)
        .onTrue(Commands.runOnce(() -> driveSubsystem.setAllWheelAngles(0.0), driveSubsystem));
    button(XboxController.Button.kBack.value)
        .onTrue(Commands.runOnce(() -> driveSubsystem.setAllWheelAngles(0.0), driveSubsystem));
    // Right stick click: hold modules at initial straight position.
    button(XboxController.Button.kRightStick.value)
        .whileTrue(Commands.run(() -> driveSubsystem.setAllWheelAngles(0.0), driveSubsystem));

    // Toggle B: press once to run shooter + tower + conveyor, press again to stop.
    button(XboxController.Button.kB.value)
        .toggleOnTrue(Commands.startEnd(
            () -> shooterSubsystem.setShooterPower(0.85),
            () -> shooterSubsystem.stop(),
            shooterSubsystem));

    // Press X to toggle intake motor (CAN 16) at 0.6 power.
    button(XboxController.Button.kX.value)
        .onTrue(Commands.runOnce(
            () -> shooterSubsystem.toggleIntakeOnly(0.6),
            shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return selected auto command, or null if none selected
    return autoChooser.getSelected();
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }
}
