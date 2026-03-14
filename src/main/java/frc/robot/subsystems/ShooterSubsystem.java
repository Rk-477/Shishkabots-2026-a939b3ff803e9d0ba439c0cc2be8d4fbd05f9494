package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import util.Logger;

/**
 * Subsystem for controlling the shooter mechanism (single motor)
 */
public class ShooterSubsystem extends SubsystemBase {
    // Shooter states - commented out as we don't need state management
    // public enum ShooterState {
    //     NO_CORAL,           // No coral in shooter, motor stopped
    //     READY_TO_INTAKE,    // Motor spinning at intake velocity, waiting for coral
    //     CORAL_INSIDE,       // Coral inside shooter, motor stopped
    //     SHOOT_CORAL         // Shooting coral, motor at shooting velocity
    // }

    private final SparkMax shooterMotorLeft;
    private final SparkMax shooterMotorRight;
    private final SparkMax towerMotor;
    private final SparkMax conveyorMotor;
    private final SparkMax intakeMotor;

    // Closed loop controllers for PID velocity control
    private final SparkClosedLoopController leftClosedLoop;
    private final SparkClosedLoopController rightClosedLoop;
    private final SparkClosedLoopController towerClosedLoop;


    
    // Feed power into shooter wheels. Lower values reduce "pop-up" at entry and flatten flight path.
    private static final double TOWER_POWER = .9;
    private static final double CONVEYOR_POWER = 0.75;

    // PID constants for shooter velocity control - aggressive tuning for faster response
    private static final double SHOOTER_P = 0.01;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.0;

      private static final double TOWER_P = 0.1;
    private static final double TOWER_I = 0.0;
    private static final double TOWER_D = 0.0;
    
    private static final double SHOOTER_FF = 0.00019; // Feedforward for SparkMax built-in
    private static final double TOWER_FF = .00019;
    
    private static final double SHOOTER_OUTPUT_SCALE = 1.00; // Full output scaling for max speed
    private static final double Tower_OUTPUT_SCALE = 1.00; // Full output scaling for max speed


    // WPILib SimpleMotorFeedforward for proper feedforward control
    // Aggressive tuning for faster spin-up and higher velocity
    private static final double FF_kS = 0.2;     // Static friction (volts) - higher for faster startup
    private static final double FF_kV = 0.118;   // Velocity gain - slightly lower to allow higher speeds
    private static final double FF_kA = 0.01;    // Acceleration gain for faster spin-up
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FF_kS, FF_kV, FF_kA);

    // Shooter wheel configuration
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES / 2.0);

    // Soft limits for safety (RPM)
    private static final double MAX_VELOCITY_RPM = 6500;
    private static final double MIN_VELOCITY_RPM = 0;


    // Target velocity for shooter (RPM)
    // Slightly faster wheel speed while keeping feed gentler for a flatter shot.
    private static final double SHOOTING_VELOCITY_RPM = 6200;
    private static final double INTAKE_VELOCITY_RPM = 2000;
    private double targetVelocity = 0;

    // Color sensor for game piece detection
    // private ColorSensorV3 colorSensor;
    // private static final int PROXIMITY_THRESHOLD = 100; // Adjust based on testing
    // private int lastProximity = 0;
    // private boolean hasColorSensor = false;

    // State management - commented out as we don't need state management
    // private ShooterState currentState = ShooterState.NO_CORAL;
    // private final Timer stateTimer = new Timer();
    // private static final double INTAKE_TIMEOUT = 10; // seconds to wait for coral to be fully inside

    // Motor configuration constants
    private static final double L4_SHOOTING_POWER = 0.90;
    private static final double SHOOTING_POWER = 0.90;
    private static final double INTAKE_POWER = 0.6;
    private static final int TOWER_CURRENT_LIMIT = 40; // Amps
    private static final int CONVEYOR_CURRENT_LIMIT = 15; // Amps
    private static final int SHOOTER_CURRENT_LIMIT = 35; // Amps
    private static final int INTAKE_CURRENT_LIMIT = 30; // Amps
    private static final double SHOOT_DURATION = 2.0; // seconds

    // telemetry timer
    private int periodicCounter = 0;
    private boolean intakeOnlyEnabled = false;

    public ShooterSubsystem(int shooterLeftCanId, int shooterRightCanId, int towerCanId, int conveyorCanId, int intakeCanId) {
        shooterMotorLeft = new SparkMax(shooterLeftCanId, MotorType.kBrushless);
        shooterMotorRight = new SparkMax(shooterRightCanId, MotorType.kBrushless);
        towerMotor = new SparkMax(towerCanId, MotorType.kBrushless);
        conveyorMotor = new SparkMax(conveyorCanId, MotorType.kBrushless);
        intakeMotor = new SparkMax(intakeCanId, MotorType.kBrushless);

        // Get closed loop controllers for PID control
        leftClosedLoop = shooterMotorLeft.getClosedLoopController();
        rightClosedLoop = shooterMotorRight.getClosedLoopController();
        towerClosedLoop = towerMotor.getClosedLoopController();

        // Configure shooter left motor with PID
        SparkMaxConfig shooterLeftConfig = new SparkMaxConfig();
        shooterLeftConfig
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(SHOOTER_CURRENT_LIMIT)
            .voltageCompensation(12.0);
        shooterLeftConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(SHOOTER_P, SHOOTER_I, SHOOTER_D)
            .velocityFF(SHOOTER_FF)
            .outputRange(-1, 1);

        shooterMotorLeft.configure(
            shooterLeftConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Configure shooter right motor with PID
        SparkMaxConfig shooterRightConfig = new SparkMaxConfig();
        shooterRightConfig
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(SHOOTER_CURRENT_LIMIT)
            .voltageCompensation(12.0);
        shooterRightConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(SHOOTER_P, SHOOTER_I, SHOOTER_D)
            .velocityFF(SHOOTER_FF)
            .outputRange(-1, 1);

        shooterMotorRight.configure(
            shooterRightConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Configure tower motor
        SparkMaxConfig towerConfig = new SparkMaxConfig();
        towerConfig
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(TOWER_CURRENT_LIMIT);
            towerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(TOWER_P, TOWER_I, TOWER_D)
            .velocityFF(TOWER_FF)
            .outputRange(-1, 1);
            

        towerMotor.configure(
            towerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Configure conveyor motor
        SparkMaxConfig conveyorConfig = new SparkMaxConfig();
        conveyorConfig
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(CONVEYOR_CURRENT_LIMIT)
            .openLoopRampRate(0.05);

        conveyorMotor.configure(
            conveyorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Configure intake motor
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(INTAKE_CURRENT_LIMIT)
            .openLoopRampRate(0.05);

        intakeMotor.configure(
            intakeConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Initialize motors stopped
        stopMotor();

        Logger.log("Shooter subsystem initialized");
    }

    // /**
    //  * Prepare the shooter to intake a coral
    //  */
    // public void prepareForIntake() {
    //     if (currentState == ShooterState.NO_CORAL) {
    //         Logger.log("Preparing shooter for intake");
    //         setMotorPower(INTAKE_POWER);
    //         currentState = ShooterState.READY_TO_INTAKE;
    //         stateTimer.reset();
    //         stateTimer.start();
    //     }
    // }

    // /**
    //  * Shoot the coral if one is inside the shooter
    //  */
    // public void shootCoral() {
    //     if (currentState == ShooterState.CORAL_INSIDE) {
    //         Logger.log("Shooting coral");
    //         setMotorPower(SHOOTING_POWER);
    //         currentState = ShooterState.SHOOT_CORAL;
    //         stateTimer.reset();
    //         stateTimer.start();
    //     } else {
    //         Logger.log("Cannot shoot - no coral inside shooter");
    //     }
    // }

    /**
     * Fine-tune the shooter intake at a slower speed
     * Used when coral doesn't go in fully and needs adjustment
     */
    public void fineTuneIntake(double power) {
        Logger.log("Fine tuning shooter intake");
        setMotorPower(power);
        // Don't change the state - this can be called from multiple states
    }

    // public void shootHighestLevelCoral() {
    //     if (currentState == ShooterState.CORAL_INSIDE) {
    //         Logger.log("Shooting coral to highest level");
    //         shooterMotorLeft.set(L4_SHOOTING_POWER);
    //         shooterMotorRight.set(L4_SHOOTING_POWER);
    //         towerMotor.set(TOWER_POWER);
    //         conveyorMotor.set(CONVEYOR_POWER);
    //         stateTimer.reset();
    //         stateTimer.start();
    //     } else {
    //         Logger.log("Cannot shoot - no coral inside shooter");
    //     }
    // }

    // public void shootBottomLevelCoral() {
    //     if (currentState == ShooterState.CORAL_INSIDE) {
    //         Logger.log("Shooting coral to bottom level");
    //         shooterMotorLeft.set(SHOOTING_POWER - 0.1);
    //         shooterMotorRight.set(SHOOTING_POWER - 0.1);
    //         towerMotor.set(TOWER_POWER);
    //         conveyorMotor.set(CONVEYOR_POWER);
    //         stateTimer.reset();
    //         stateTimer.start();
    //     } else {
    //         Logger.log("Cannot shoot - no coral inside shooter");
    //     }
    // }

    /**
     * Emergency stop for the shooter
     */
    public void emergencyStop() {
        stopMotor();
        Logger.log("Emergency stop triggered");
    }

    /**
     * Set the motor power using open-loop control
     * @param percentOutput Target percentage output (-1.0 to 1.0)
     */
    private void setMotorPower(double percentOutput) {
        double scaledOutput = percentOutput * SHOOTER_OUTPUT_SCALE;
        Logger.log("Setting shooter power to " + scaledOutput + " (scaled)");
        shooterMotorLeft.set(scaledOutput);
        shooterMotorRight.set(scaledOutput);
        towerMotor.set(TOWER_POWER);
        conveyorMotor.set(CONVEYOR_POWER);
    }

    /**
     * Stop the shooter motor
     */
    private void stopMotor() {
        Logger.log("Stopping shooter, tower, and conveyor motors");
        shooterMotorLeft.stopMotor();
        shooterMotorRight.stopMotor();
        towerMotor.stopMotor();
        conveyorMotor.stopMotor();
        intakeMotor.stopMotor();
    }

    /**
     * Public method to stop the motor
     * Can be called from commands
     */
    public void stop() {
        stopMotor();
        intakeOnlyEnabled = false;
    }

    /**
     * Checks if a game piece has entered the shooter using the color sensor's proximity reading
     * @return true if a game piece is detected at the entry of the shooter, false if no sensor
     */
    public boolean hasGamePieceEntered() {
        // Color sensor disabled
        return false;
        // if (!hasColorSensor) {
        //     return false;
        // }

        // int proximity = colorSensor.getProximity();
        // boolean isClose = proximity > PROXIMITY_THRESHOLD;

        // // If we detect a sudden increase in proximity, a game piece likely entered
        // if (isClose && lastProximity <= PROXIMITY_THRESHOLD) {
        //     var detectedColor = colorSensor.getColor();
        //     Logger.log(String.format("Coral detected! Color: R=%.2f, G=%.2f, B=%.2f, Proximity=%d",
        //         detectedColor.red, detectedColor.green, detectedColor.blue, proximity));
        // }

        // lastProximity = proximity;
        // return isClose;
    }

    /**
     * Checks if a game piece has exited the shooter by checking if proximity drops after being high
     * @return true if a game piece is detected leaving the shooter, false if no sensor
     */
    public boolean hasGamePieceExited() {
        // Color sensor disabled
        return false;
        // if (!hasColorSensor) {
        //     return false;
        // }

        // int proximity = colorSensor.getProximity();
        // boolean hasExited = lastProximity > PROXIMITY_THRESHOLD && proximity <= PROXIMITY_THRESHOLD;
        // Logger.log("proximity sensor value = " + proximity + " hasExited = " + hasExited);
        // if (hasExited) {
        //     Logger.log("Coral has exited the shooter");
        // }

        // lastProximity = proximity;
        // return hasExited;
    }

    // /**
    //  * Get the current state of the shooter
    //  * @return Current shooter state
    //  */
    // public ShooterState getState() {
    //     return currentState;
    // }

    @Override
    public void periodic() {
        // Update telemetry
        if (periodicCounter++ % 50 == 0) {
            updateTelemetry();
        }
    }

    private void updateTelemetry() {
        // Calculate voltages
        double shooterLeftVoltage = shooterMotorLeft.getBusVoltage() * shooterMotorLeft.getAppliedOutput();
        double shooterRightVoltage = shooterMotorRight.getBusVoltage() * shooterMotorRight.getAppliedOutput();
        double towerVoltage = towerMotor.getBusVoltage() * towerMotor.getAppliedOutput();
        double conveyorVoltage = conveyorMotor.getBusVoltage() * conveyorMotor.getAppliedOutput();

        // Get velocities
        double leftVelocity = shooterMotorLeft.getEncoder().getVelocity();
        double rightVelocity = shooterMotorRight.getEncoder().getVelocity();

        // Send to SmartDashboard (visible in Shuffleboard)
        SmartDashboard.putNumber("Shooter/LeftVoltage", shooterLeftVoltage);
        SmartDashboard.putNumber("Shooter/RightVoltage", shooterRightVoltage);
        SmartDashboard.putNumber("Shooter/TowerVoltage", towerVoltage);
        SmartDashboard.putNumber("Shooter/ConveyorVoltage", conveyorVoltage);
        SmartDashboard.putNumber("Shooter/IntakeVoltage", intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/LeftCurrent", shooterMotorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/RightCurrent", shooterMotorRight.getOutputCurrent());

        // PID telemetry - velocity tracking
        SmartDashboard.putNumber("Shooter/TargetVelocityRPM", targetVelocity);
        SmartDashboard.putNumber("Shooter/LeftVelocityRPM", leftVelocity);
        SmartDashboard.putNumber("Shooter/RightVelocityRPM", rightVelocity);
        SmartDashboard.putNumber("Shooter/VelocityErrorRPM", targetVelocity - getShooterVelocity());

        // Tangential velocity (exit speed of game piece)
        SmartDashboard.putNumber("Shooter/TangentialVelocityMPS", getTangentialVelocityMPS());
        SmartDashboard.putNumber("Shooter/TangentialVelocityFPS", getTangentialVelocityMPS() * 3.281); // Convert to ft/s
    }

    /**
     * Set shooter power directly (open-loop control)
     * @param power Power level (-1.0 to 1.0)
     */
    public void setShooterPower(double power) {
        double scaledPower = power * SHOOTER_OUTPUT_SCALE;
        Logger.log("Setting shooter power to " + scaledPower + " (scaled)");
        shooterMotorLeft.set(scaledPower);
        shooterMotorRight.set(scaledPower);
        towerMotor.set(TOWER_POWER);
        conveyorMotor.set(CONVEYOR_POWER);
    }

    /**
     * Run intake motor + conveyor together (used for loading game pieces).
     * @param power Open loop power [-1, 1]
     */
    public void runIntakeAndConveyor(double power) {
        intakeMotor.set(power);
        conveyorMotor.set(power);
    }

    /**
     * Run intake + conveyor at default intake power.
     */
    public void runIntakeAndConveyor() {
        runIntakeAndConveyor(INTAKE_POWER);
    }

    /**
     * Run only the intake motor.
     * @param power Open loop power [-1, 1]
     */
    public void runIntakeOnly(double power) {
        intakeMotor.set(power);
        intakeOnlyEnabled = true;
    }

    /**
     * Run intake motor at default intake power.
     */
    public void runIntakeOnly() {
        runIntakeOnly(INTAKE_POWER);
    }

    /**
     * Stop only intake and conveyor motors.
     */
    public void stopIntakeAndConveyor() {
        intakeMotor.stopMotor();
        conveyorMotor.stopMotor();
    }

    /**
     * Stop only the intake motor.
     */
    public void stopIntakeOnly() {
        intakeMotor.stopMotor();
        intakeOnlyEnabled = false;
    }

    /**
     * Toggle intake-only mode on the intake motor.
     * @param power Intake motor power when enabling.
     */
    public void toggleIntakeOnly(double power) {
        if (intakeOnlyEnabled) {
            stopIntakeOnly();
        } else {
            runIntakeOnly(power);
        }
    }

    /**
     * Set shooter velocity using PID control
     * @param velocityRPM Target velocity in RPM
     */
    public void setShooterVelocity(double velocityRPM) {
        double scaledVelocityRPM = velocityRPM * SHOOTER_OUTPUT_SCALE;
        targetVelocity = scaledVelocityRPM;
        Logger.log("Setting shooter velocity to " + scaledVelocityRPM + " RPM (scaled)");
        leftClosedLoop.setSetpoint(scaledVelocityRPM, ControlType.kVelocity);
        rightClosedLoop.setSetpoint(scaledVelocityRPM, ControlType.kVelocity);
        towerClosedLoop.setSetpoint(scaledVelocityRPM, ControlType.kVelocity);
        conveyorMotor.set(CONVEYOR_POWER);
    }

    public void setTowerVelocity (double velocityRPM) {
        double scaledVelocityRPM = velocityRPM * Tower_OUTPUT_SCALE;
        Logger.log("Setting tower velocity to " + scaledVelocityRPM + " RPM (scaled)");
            towerMotor.set(TOWER_POWER);
        towerClosedLoop.setReference(scaledVelocityRPM, ControlType.kVelocity);
    }

    /**
     * Run shooter at shooting speed using PID
     */
    public void shootWithPID() {
        setShooterPower(.85);
    
    }

    /**
     * Run shooter at intake speed using PID
     */
    public void intakeWithPID() {
        setShooterVelocity(-INTAKE_VELOCITY_RPM);
    }

    /**
     * Get the current shooter velocity (average of both motors)
     * @return Current velocity in RPM
     */
    public double getShooterVelocity() {
        double leftVel = shooterMotorLeft.getEncoder().getVelocity();
        double rightVel = shooterMotorRight.getEncoder().getVelocity();
        return (leftVel + rightVel) / 2.0;
    }

    /**
     * Check if shooter is at target velocity (within tolerance)
     * @param toleranceRPM Acceptable error in RPM
     * @return true if at target velocity
     */
    public boolean isAtTargetVelocity(double toleranceRPM) {
        return Math.abs(getShooterVelocity() - targetVelocity) < toleranceRPM;
    }

    /**
     * Set shooter velocity using WPILib SimpleMotorFeedforward
     * Uses proper feedforward model: voltage = kS * sign(velocity) + kV * velocity
     * @param velocityRPM Target velocity in RPM
     */
    public void setShooterVelocityFF(double velocityRPM) {
        // Apply soft limits
        velocityRPM = Math.max(-MAX_VELOCITY_RPM, Math.min(MAX_VELOCITY_RPM, velocityRPM));
        velocityRPM = velocityRPM * SHOOTER_OUTPUT_SCALE;
        targetVelocity = velocityRPM;

        // Convert RPM to rotations per second for SimpleMotorFeedforward
        double velocityRotPerSec = velocityRPM / 60.0;

        // Calculate feedforward voltage using WPILib SimpleMotorFeedforward
        // Then convert to duty cycle (divide by 12V nominal)
        double ffVoltage = feedforward.calculate(velocityRotPerSec);
        double dutyCycle = ffVoltage / 12.0;

        // Clamp output to valid range
        dutyCycle = Math.max(-1.0, Math.min(1.0, dutyCycle));

        Logger.log("Shooter FF: target=" + velocityRPM + " RPM, voltage=" +
                   String.format("%.2f", ffVoltage) + "V, duty=" + String.format("%.2f", dutyCycle));
        shooterMotorLeft.set(dutyCycle);
        shooterMotorRight.set(dutyCycle);
        towerMotor.set(TOWER_POWER);
        conveyorMotor.set(CONVEYOR_POWER);
    }

    /**
     * Get the tangential velocity at the wheel surface (exit speed of game piece)
     * @return Linear velocity in meters per second
     */
    public double getTangentialVelocityMPS() {
        // Convert RPM to radians per second, then multiply by wheel radius
        double radiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(getShooterVelocity());
        return radiansPerSecond * WHEEL_RADIUS_METERS;
    }

    /**
     * Command to set shooter to a specific velocity
     * @param velocityRPM Target velocity in RPM
     * @return Command that runs the shooter at the specified velocity
     */
    public Command setSpeedCommand(double velocityRPM) {
        return Commands.run(() -> setShooterVelocityFF(velocityRPM), this);
    }

    /**
     * Command to spin up the shooter to shooting speed
     * @return Command that spins up the shooter
     */
    public Command spinUpCommand() {
        return setSpeedCommand(SHOOTING_VELOCITY_RPM);
    }

    /**
     * Command to stop the shooter
     * @return Command that stops the shooter
     */
    public Command stopCommand() {
        return Commands.runOnce(() -> stop(), this);
    }
}
