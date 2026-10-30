package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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
public class DmShooterSubsystem extends StealthSubsystem {

    /** The motor that drives the shooter. */
    private final DcMotorEx shooterMotor1;

    private final DcMotorEx shooterMotor2;

    /** Ticks per revolution for the motor. */
    private static final double MOTOR_TICKS_REV = 28;
    /** The gear ratio between the motor and the shooter. */
    private static final double GEAR_RATIO = 1;
    /** Telemetry used for debugging and robot feedback. */
    private final Telemetry telemetry = StealthOpMode.telemetry;
    /** Timer to track how long the shooter has been active. */
    private final ElapsedTime shootTimer = new ElapsedTime();

    /** RPM tolerance for considering the target reached. */
    private static final double VELOCITY_TOLERANCE_HIGH = 150;

    /** RPM tolerance for considering the target reached. */
    private static final double VELOCITY_TOLERANCE_LOW = -50;


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

    private static final double FLOAT_DEFAULT_POWER = .4;
    /** Minimum distance for lookup tables. */
    private static final double MIN_DISTANCE = 35;
    /** Maximum distance for lookup tables. */
    private static final double MAX_DISTANCE = 75;

    /** Additional RPM added to the target during shooting. */
    private double boostRpm = 0;

    /** Voltage sensor to monitor battery level and compensate PIDF. */
    private final VoltageSensor voltageSensor;
    /** Nominal voltage for feedforward calculation (12V baseline). */
    private final double NOMINAL_VOLTAGE = 13.1;

    private double baseKp = 1.0;
    private double baseKi = 0.2;
    private double baseKd = 0.0;
    private double baseKf = 12.16;
    /** Scaler for voltage compensation effect on kF (1.0 = 100% compensation, 0.5 = 50%, 0.0 = disabled). */
    private double kfScaler = .8;

    private InterpLUT rangeToRpm;

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
    public DmShooterSubsystem(HardwareMap map, RobotSystem robot) {
        this.robot = robot;
        voltageSensor = map.voltageSensor.iterator().next();
        hood = new HoodSubsystem(map);
        shooterMotor1 = map.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = map.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setVelocityPIDFCoefficients(baseKp, baseKi, baseKd, baseKf);
        shooterMotor1.setPositionPIDFCoefficients(5.0);
        shooterMotor1.setTargetPositionTolerance(20);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setVelocityPIDFCoefficients(baseKp, baseKi, baseKd, baseKf);
        shooterMotor2.setPositionPIDFCoefficients(5.0);
        shooterMotor2.setTargetPositionTolerance(20);
        //shooterMotor1.pi
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
        rangeToRpm.add(45.0, 3620);
        rangeToRpm.add(50.0, 3610);
        rangeToRpm.add(55.0, 3675);
        rangeToRpm.add(61.0, 3700);
        rangeToRpm.add(62.0, 3710);
        rangeToRpm.add(69.0, 4025);
        rangeToRpm.add(MAX_DISTANCE + 0.1, 4000);
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

        double currentVoltage = getBatteryVoltage();
//        telemetry.addData("Battery Voltage", "%.2f V", currentVoltage);
//        telemetry.addData("Target RPM", targetRpm);
//        telemetry.addData("Current RPM", currentRpm);
//        telemetry.addData("Last Distance", lastDistance);
//        telemetry.addData("Hood Angle", hood.getHoodAngle());


        // if we are shooting or waiting to shoot
        if (isActiveShot()) {

            // Apply battery voltage compensation to feedforward (kF)
            updateVoltageCompensation(currentVoltage);

            // set the hood angle to the distance
            hood.setHoodAngle(lastDistance);

            // set the target RPMs for the distance
            setRpmFromRange(lastDistance);

            // Do not run the intake/transfer until the shooter is ready
            if (shootState == ShootState.WAITING_TO_SHOOT) {
                setFeeding(false);
            }

            // Handle automatic intake control based on shooting readiness
            if (isShootReady()) {
                shootState = ShootState.SHOOTING;
                robot.getGateSubsystem().openGate();
                setFeeding(true);
            } else {
                shootState = ShootState.WAITING_TO_SHOOT;
                setFeeding(false);
                robot.getGateSubsystem().closeGate();
                }
            }


    }

    /**
     * Gets the current battery voltage from the hardware map's voltage sensor.
     *
     * @return Battery voltage in Volts.
     */
    public double getBatteryVoltage() {
        if (voltageSensor != null) {
            return voltageSensor.getVoltage();
        }
        return NOMINAL_VOLTAGE;
    }

    /**
     * Updates the motor PIDF coefficients with voltage-compensated feedforward (kF).
     *
     * @param currentVoltage Current measured battery voltage.
     */
    private void updateVoltageCompensation(double currentVoltage) {
        if (currentVoltage > 8.0 && currentVoltage < NOMINAL_VOLTAGE) {
            double voltageDelta = (NOMINAL_VOLTAGE / currentVoltage) - 1.0;
            double compensatedKf = baseKf * (1.0 + (voltageDelta * kfScaler));
            shooterMotor1.setVelocityPIDFCoefficients(baseKp, baseKi, baseKd, compensatedKf);
            shooterMotor2.setVelocityPIDFCoefficients(baseKp, baseKi, baseKd, compensatedKf);
            telemetry.addData("Compensated KF", "%.4f", compensatedKf);
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
        return (currentRpm - targetRpm >= VELOCITY_TOLERANCE_LOW) && (currentRpm - targetRpm <= VELOCITY_TOLERANCE_HIGH)
                ;//|| (curMs > MAX_SHOOT_TIME_MS);
    }

    /**
     * Updates and returns the current RPM.
     *
     * @return the current RPM.
     */
    public double getCurrentRpm() {
        return tpsToRpm(getVelocity());
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
        baseKp = kp;
        baseKi = ki;
        baseKd = kd;
        baseKf = kf;
        updateVoltageCompensation(getBatteryVoltage());
    }

    /**
     * Gets the voltage compensation kF scaler.
     *
     * @return The kF voltage compensation scaler.
     */
    public double getKfScaler() {
        return kfScaler;
    }

    /**
     * Sets the voltage compensation kF scaler.
     *
     * @param kfScaler Scaler factor for voltage compensation (1.0 = full compensation, 0.5 = half, 0.0 = none).
     */
    public void setKfScaler(double kfScaler) {
        this.kfScaler = kfScaler;
        updateVoltageCompensation(getBatteryVoltage());
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
        shootState = ShootState.NOT_SHOOTING;
        customDistance = 0;
        shooterMotor1.setPower(floatPower);
        shooterMotor2.setPower(floatPower);
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

    public void setDistance(double distance) {
        customDistance = distance;
    }

    /**
     * Sets the target RPM, applying boost and clamping to valid ranges.
     *
     * @param rpm The target RPM.
     */
    public void setRpm(double rpm) {
        if (rpm == 0) {
            stop();
            return;
        }

        // Apply boost if RPM is non-zero
        rpm += boostRpm;

        // Clamp the RPM to the allowable min/max range to prevent motor damage or unexpected behavior.
        rpm = Math.clamp(rpm, MIN_RPM, MAX_RPM);

        // prevents constant resets of the RPM controller
        if (Math.abs(targetRpm - rpm) >= VELOCITY_CHANGE_TOLERANCE) {
            shootTimer.reset();
        }

        targetRpm = rpm;
        setVelocity(rpmToTps(targetRpm));
    }

    /**
     * Sets the idle (float) power applied when not shooting.
     *
     * @param power The power level (-1.0 to 1.0).
     */
    public void setFloatPower(double power) {
        floatPower = power;
    }

    public void toggleFloatPower(){
        floatPower = floatPower > 0 ? 0 : FLOAT_DEFAULT_POWER;
        stop();
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
     * @param velocity The power level.
     */

    private void setVelocity(double velocity) {
        shooterMotor1.setVelocity(velocity);
        shooterMotor2.setVelocity(velocity);
    }

    /**
     * Gets the motor velocity in ticks per second.
     *
     * @return The motor velocity.
     */
    private double getVelocity() {

        return (shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2;
    }

    /**
     * Converts Ticks Per Second (TPS) to RPM.
     *
     * @param tps Ticks per second.
     * @return RPM.
     */
    private double tpsToRpm(double tps) {
        return Math.abs(tps  / MOTOR_TICKS_REV * 60) / GEAR_RATIO;
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
        if (shoot && shootState == ShootState.NOT_SHOOTING) {
            shootTimer.reset();
            shootState = ShootState.WAITING_TO_SHOOT;
        } else if (shootState != ShootState.NOT_SHOOTING){
            stop();
        }
    }

    /**
     * Returns a command that shoots at a specific distance.
     *
     * @param distance The distance to the target.
     * @return A command that performs the shoot action.
     */
    public Command shootCmd(double distance) {
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
     * with override distance
     * @return A command that sets the start shoot flag to true.
     */
    public Command startShootCmd(double overrideDistance) {
        return runOnce(() -> {
            this.setDistance(overrideDistance);
            setStartShoot(true);
        });
    }

    /**
     * Returns a command that starts the shooting sequence.
     *
     * @return A command that sets the start shoot flag to true.
     */
    public Command startShootCmd() {
        return startShootCmd(0);
    }


    /**
     * Returns a command that stops the shooting sequence.
     *
     * @return A command that sets the start shoot flag to false.
     */
    public Command stopShootCmd() {
        return runOnce(() -> setStartShoot(false));
    }

    /**
     * Returns acommand that adjusts the boost RPM.
     *
     * @param amount The amount to adjust.
     * @return A command that adjusts the boost RPM.
     */
    public Command adjustBoostRpmCmd(double amount) {
        return runOnce(() -> adjustBoostRpm(amount));
    }

    /**
     * Returns a command that shoots for a specific duration.
     *
     * @param durationMs The duration in milliseconds.
     * @return A command that shoots for the specified duration.
     */
    public Command shootForTimeCmd(long durationMs) {
        return startShootCmd(0)
                .andThen(new WaitCommand(durationMs))
                .andThen(stopShootCmd());
    }

    /**
     * Returns a command that waits for shooter spinup and then shoots for given duration.
     *
     * @param durationMs The duration in milliseconds to wait after becoming ready.
     * @return A command that shoots until ready and then waits.
     */
    public Command shootWhenReadyCmd(long durationMs) {
        return shootWhenReadyCmd(durationMs, 0);
    }

    /**
     * Returns a command that waits for shooter spinup and then shoots for given duration and distance
     * @param durationMs The duration in milliseconds to wait after becoming ready.
     * @param distanceOverride Override the distance to shoot to the goal good for not needing to calc distance
     * @return A command that shoots until ready and then waits.
     */
    public Command shootWhenReadyCmd(long durationMs, double distanceOverride) {
        return startShootCmd(distanceOverride)
                .andThen(new WaitUntilCommand(this::isShootReady))
                .andThen(new WaitCommand(durationMs))
                .andThen(stopShootCmd());
    }


}