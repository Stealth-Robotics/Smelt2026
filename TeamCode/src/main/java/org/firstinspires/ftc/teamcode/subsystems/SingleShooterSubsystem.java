package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

/**
 * Subsystem that controls a single shooter motor using a PIDF controller.
 * It supports RPM-based control, distance-to-RPM lookups, and a "boost" feature.
 *
 * <p>This subsystem manages a single DcMotorEx and uses a PIDF controller to maintain
 * a target RPM. It also includes lookup tables for converting distance to RPM
 * and hood angle to RPM.</p>
 */
public class SingleShooterSubsystem extends StealthSubsystem {

    /** The motor that drives the shooter. */
    private final DcMotorEx shooterMotor1;
    /** Ticks per revolution for the motor. */
    private static final double MOTOR_TICKS_REV = 28;
    /** The gear ratio between the motor and the shooter. */
    private static final double GEAR_RATIO = 1;
    /** Telemetry used for debugging and robot feedback. */
    private final Telemetry telemetry = StealthOpMode.telemetry;
    /** Timer to track how long the shooter has been active. */
    private final ElapsedTime shootTimer = new ElapsedTime();

    /** RPM tolerance for considering the target reached. */
    private static final double VELOCITY_TOLERANCE_HIGH = 100;

    /** Threshold for RPM changes to reset the PID controller. */
    private static final double VELOCITY_CHANGE_TOLERANCE = 50;
    /** Maximum allowable RPM. */
    public static final double MAX_RPM = 5400;
    /** Minimum allowable RPM (negative for reverse). */
    public static final double MIN_RPM = -4000;

    /** Minimum time the shooter must run before it's considered ready. */
    private static final double MIN_SHOOT_TIME_MS = 200;
    /** Maximum time the shooter can run before it's considered timed out. */
    private static final double MAX_SHOOT_TIME_MS = 2000;
    /** The current target RPM. */
    private double targetRpm = 0;
    /** The current measured RPM. */
    private double currentRpm = 0;
    /** Last known distance, used to resume shooting if no distance is provided. */
    private double lastDistance = 0;

    /** Default distance used when no distance is specified. */
    private static final double DEFAULT_SHOOT_DISTANCE = 40;

    /** The "float" power applied when not actively shooting. */
    private double floatPower = 0;
    /** Minimum distance for lookup tables. */
    private static final double MIN_DISTANCE = 35;
    /** Maximum distance for lookup tables. */
    private static final double MAX_DISTANCE = 75;

    /** Additional RPM added to the target during shooting. */
    private double boostRpm = 0;

    /** PIDF controller for maintaining target RPM. */
    private static final PIDFController shooterController = new PIDFController(
            0.000003,
            0.000000,
            0.0,
            0.000408);
    /** Interpolation table for Distance -> RPM. */
    private InterpLUT rangeToRpm;
    /** Interpolation table for Distance -> Hood Angle. */
    private InterpLUT rangeToHoodAngle;

    /** The robot system instance containing other subsystems. */
    private final RobotSystem robot;

    private final HoodSubsystem hood;

    private final IntakeSubsystem intake;

    public enum ShootState {
        NOT_SHOOTING,
        WAITING_TO_SHOOT,
        SHOOTING,
    }

    private ShootState shootState = ShootState.NOT_SHOOTING;

    // Allows for autonomous shooting overrides the distance calculation
    private double customDistance = 0;

    /**
     * Constructs a new SingleShooterSubsystem.
     *
     * @param map     The hardware map for accessing robot components.
     * @param robot   The main robot system instance.
     */
    public SingleShooterSubsystem(HardwareMap map, RobotSystem robot) {
        this.robot = robot;
        hood = new HoodSubsystem(map);
        shooterMotor1 = map.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake = robot.getIntake();
        buildRangeTable();
    }

    /**
     * Initializes the interpolation tables for distance-based lookups.
     */
    public void buildRangeTable() {
        rangeToRpm = new InterpLUT();
        rangeToRpm.add(MIN_DISTANCE - 0.1, 3550);
        rangeToRpm.add(40.0, 3600);
        rangeToRpm.add(45.0, 3625);
        rangeToRpm.add(50.0, 3630);
        rangeToRpm.add(55.0, 3650);
        rangeToRpm.add(69.0, 4000);
        rangeToRpm.add(MAX_DISTANCE + 0.1, 4100);
        rangeToRpm.createLUT();
    }

    /**
     * Periodic update for the subsystem. Handles PID control and state transitions.
     */
    @Override
    public void periodic() {
        CameraSubsystem camera = robot.getCamera();

        // Update distance to goal regardless of shoot state
        LLResult result = camera.getLastResult();
        if (result != null && result.isValid()) {
            lastDistance = camera.calcGoalDistanceByTa(result);
        }

        if (customDistance != 0) {
            lastDistance = customDistance;
        }else if (lastDistance == 0) {
            lastDistance = DEFAULT_SHOOT_DISTANCE;
        }

        telemetry.addData("Target RPM", targetRpm);
        telemetry.addData("Current RPM", currentRpm);
        telemetry.addData("Last Distance", lastDistance);
        telemetry.addData("Hood Angle", hood.getHoodAngle());


        // if we are shooting or waiting to shoot
        if (isActiveShot()) {

            // set the hood angle to the distance
            hood.setHoodAngle(lastDistance);

            // set the target RPMs for the distance
            setRpmFromRange(lastDistance);

            // Apply PID control while an active shot is in progress
            double calculatedPower = shooterController.calculate(getVelocity());
            setPower(calculatedPower);

            // Do not run the intake/transfer until the shooter is ready
            if (shootState == ShootState.WAITING_TO_SHOOT) {
                setFeeding(false);
            }

            // Handle automatic intake control based on shooting readiness
            if (isShootReady()) {
                shootState = ShootState.SHOOTING;
                setFeeding(true);
            } else {
                shootState = ShootState.WAITING_TO_SHOOT;
            }
        }
        else {
            setFeeding(false);
        }
    }

    public HoodSubsystem getHood(){
        return hood;
    }

    private void setFeeding(boolean feed) {
        intake.setAutoIntake(feed);
        // TODO: add the stopper servo
    }

    /**
     * Checks if the shooter has reached its target RPM and has been running long enough.
     *
     * @return true if RPM is within tolerance or timeout has been reached.
     */
    public boolean isShootReady() {
        if (targetRpm == 0) {
            return false;
        }

        currentRpm = getCurrentRpm();
        double difference = Math.abs(currentRpm - targetRpm);
        double curMs = shootTimer.milliseconds();

        // Prevent immediate "ready" status by requiring a minimum runtime
        if (curMs < MIN_SHOOT_TIME_MS) {
            return false;
        }

        // Ready if RPM is within tolerance
        return (difference <= VELOCITY_TOLERANCE_HIGH)
                ;//|| (curMs > MAX_SHOOT_TIME_MS);
    }

    /**
     * Updates and returns the current RPM.
     *
     * @return the current RPM.
     */
    public double getCurrentRpm() {
        return tpsToRpm(shooterMotor1.getVelocity());
    }

    /**
     * Returns the current target RPM.
     *
     * @return the target RPM.
     */
    public double getTargetRpm() {
        return targetRpm;
    }

    /**
     * Sets the PIDF parameters for the shooter controller.
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param kf Feedforward gain
     */
    public void setController(double kp, double ki, double kd, double kf) {
        shooterController.setPIDF(kp, ki, kd, kf);
    }

    /**
     * Sets the target RPM based on a provided distance.
     *
     * @param distance The distance to the target.
     */
    private void setRpmFromRange(double distance) {
        if (distance == 0) {
            stop();
            return;
        }

        setRpm(getRpmFromRange(distance));
    }

    /**
     * Stops the motor by setting target RPM to zero and turning off intake.
     */
    public void stop() {
        customDistance = 0;
        setRpm(0);
        setFeeding(false);
    }

    /**
     * Looks up the required RPM for a given distance using the interpolation table.
     *
     * @param distance The distance to the target.
     * @return The target RPM for that distance.
     */
    public double getRpmFromRange(double distance) {
        if (distance == 0) {
            return 0;
        }

        double dist = Math.clamp(distance, MIN_DISTANCE, MAX_DISTANCE);
        return rangeToRpm.get(dist);
    }

    /**
     * Sets the target RPM, applying boost and clamping to valid ranges.
     *
     * @param rpm The target RPM.
     */
    public void setRpm(double rpm) {
        if (rpm == 0) {
            shootState = ShootState.NOT_SHOOTING;
            targetRpm = 0;
            shooterController.reset();
            setPower(floatPower);
            return;
        }

        // Apply boost if RPM is non-zero
        rpm += boostRpm;

        // Clamp the RPM to the allowable min/max range to prevent motor damage or unexpected behavior.
        rpm = Math.clamp(rpm, MIN_RPM, MAX_RPM);

        // prevents constant resets of the RPM controller
        if (Math.abs(targetRpm - rpm) >= VELOCITY_CHANGE_TOLERANCE) {
            shootTimer.reset();
            shooterController.reset();
        }

        targetRpm = rpm;
        shooterController.setSetPoint(rpmToTps(targetRpm));
    }

    /**
     * Sets the idle (float) power applied when not shooting.
     *
     * @param power The power level (-1.0 to 1.0).
     */
    public void setFloatPower(double power) {
        floatPower = power;
    }

    /**
     * Checks if the subsystem is currently actively trying to reach a target RPM.
     *
     * @return true if target RPM is non-zero.
     */
    public boolean isActiveShot() {
        return  shootState != ShootState.NOT_SHOOTING;
    }

    /**
     * Sets the motor power directly.
     *
     * @param power The power level.
     */
    private void setPower(double power) {
        shooterMotor1.setPower(power);
    }

    /**
     * Gets the motor velocity in ticks per second.
     *
     * @return The motor velocity.
     */
    private double getVelocity() {
        return shooterMotor1.getVelocity();
    }

    /**
     * Converts Ticks Per Second (TPS) to RPM.
     *
     * @param tps Ticks per second.
     * @return RPM.
     */
    private double tpsToRpm(double tps) {
        return Math.abs(tps * 60 / MOTOR_TICKS_REV) / GEAR_RATIO;
    }

    /**
     * Converts RPM to Ticks Per Second (TPS).
     *
     * @param rpm RPM.
     * @return TPS.
     */
    private double rpmToTps( double rpm) {
        return Math.abs(rpm * MOTOR_TICKS_REV / 60) * GEAR_RATIO;
    }

    /**
     * Gets the current boost RPM value.
     *
     * @return The boost RPM.
     */
    public double getBoostRpm() {
        return boostRpm;
    }

    /**
     * Sets the boost RPM value.
     *
     * @param boostRpm The boost RPM.
     */
    public void setBoostRpm(double boostRpm) {
        this.boostRpm = boostRpm;
    }

    /**
     * Resets the boost RPM to zero.
     */
    public void resetBoostRpm() {
        this.boostRpm = 0;
    }

    /**
     * Adjusts the boost RPM by a specified amount.
     *
     * @param amount The amount to adjust.
     */
    public void adjustBoostRpm(double amount) {
        this.boostRpm += amount;
    }

    /**
     * Sets the start shoot flag.
     *
     * @param shoot true if shooting should start, false otherwise.
     */
    public void setStartShoot(boolean shoot) {
        if (shoot) {
            shootTimer.reset();
            shooterController.reset();
            shootState = ShootState.WAITING_TO_SHOOT;
        } else {
            stop();
        }
    }

    /**
     * Returns a command that shoots at a specific distance.
     *
     * @param distance The distance to the target.
     * @return A command that performs the shoot action.
     */
    public Command shootCommand(double distance) {
        return runOnce(() -> {
            customDistance = distance;
            setStartShoot(true);
        });
    }

    /**
     * Returns a command that resets the boost RPM.
     *
     * @return A command that resets the boost RPM.
     */
    public Command resetBoostRpmCommand() {
        return runOnce(this::resetBoostRpm);
    }

    /**
     * Returns a command that starts the shooting sequence.
     *
     * @return A command that sets the start shoot flag to true.
     */
    public Command startShootCommand() {
        return runOnce(() -> setStartShoot(true));
    }

    /**
     * Returns a command that stops the shooting sequence.
     *
     * @return A command that sets the start shoot flag to false.
     */
    public Command stopShootCommand() {
        return runOnce(() -> setStartShoot(false));
    }

    /**
     * Returns acommand that adjusts the boost RPM.
     *
     * @param amount The amount to adjust.
     * @return A command that adjusts the boost RPM.
     */
    public Command adjustBoostRpmCommand(double amount) {
        return runOnce(() -> adjustBoostRpm(amount));
    }

    /**
     * Returns a command that shoots for a specific duration.
     *
     * @param durationMs The duration in milliseconds.
     * @return A command that shoots for the specified duration.
     */
    public Command shootForTimeCommand(long durationMs) {
        return startShootCommand()
                .andThen(new WaitCommand(durationMs))
                .andThen(stopShootCommand());
    }

    /**
     * Returns a command that shoots until the motor is ready and then waits for a specified duration.
     *
     * @param durationMs The duration in milliseconds to wait after becoming ready.
     * @return A command that shoots until ready and then waits.
     */
    public Command shootWhenReadyCommand(long durationMs) {
        return startShootCommand()
                .andThen(new WaitUntilCommand(this::isShootReady))
                .andThen(new WaitCommand(durationMs))
                .andThen(stopShootCommand());
    }
}