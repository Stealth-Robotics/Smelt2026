package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tools.PoseTracker;
import org.firstinspires.ftc.teamcode.tools.Targets;
import org.firstinspires.ftc.teamcode.tools.Trajectories;
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
public class ShooterSubsystem extends StealthSubsystem {

    private final PIDFController shooterPid = new PIDFController(
            0.002,
            0.0001,
            0.0,
            0.000202
    );

    private final double baseKf = shooterPid.getF();

    /** Adjust for lower voltage compensation boost */
    private static final double KF_SCALER = 1.0;

    /** Motor 1 is right side on control hub. */
    private final DcMotorEx shooterMotor1;

    /** Motor 2 is left side on expansion hub. */
    private final DcMotorEx shooterMotor2;

    /** Ticks per revolution for the motor. */
    private static final double MOTOR_TICKS_REV = 28;
    /** The gear ratio between the motor and the shooter. */
    private static final double GEAR_RATIO = 1.125;
    /** Telemetry used for debugging and robot feedback. */
    private final Telemetry telemetry = StealthOpMode.telemetry;
    /** Timer to track how long the shooter has been active. */
    private final ElapsedTime shootTimer = new ElapsedTime();

    /** RPM tolerance for considering the target reached. */
    private static final double VELOCITY_TOLERANCE_HIGH = 50;

    /** RPM tolerance for considering the target reached. */
    private static final double VELOCITY_TOLERANCE_LOW = -10;

    /** Maximum allowable RPM. */
    public static final double MAX_RPM = 5400;
    /** Minimum allowable RPM (negative for reverse). */
    public static final double MIN_RPM = -4000;

    /** Minimum time the shooter must run before it's considered ready. */
    private static final double MIN_SHOOT_TIME_MS = 500;
    /** Maximum time the shooter can run before it's considered timed out. */
    private static final double MAX_SHOOT_TIME_MS = 3000;
    /** The current target RPM. */
    private double targetRpm = 0;
    /** Last known distance, used to resume shooting if no distance is provided. */
    private double lastDistance = 0;

    /** Default distance used when no distance is specified. */
    public static final double DEFAULT_SHOOT_DISTANCE = 60;

    /** The "float" power applied when not actively shooting. */
    private double floatPower = 0;

    private static final double FLOAT_DEFAULT_POWER = 0.55;
    /** Minimum distance for lookup tables. */
    public static final double MIN_DISTANCE = 35;
    /** Maximum distance for lookup tables. */
    public static final double MAX_DISTANCE = 100;

    /** Additional RPM added to the target during shooting. */
    private double boostRpm = 0;

    /** Voltage sensor to monitor battery level and compensate PIDF. */
    private final VoltageSensor voltageSensor;

    /** Nominal voltage for feedforward calculation (12.5V baseline). */
    private final double NOMINAL_VOLTAGE = 12.5;

    private final InterpLUT rangeToRpm;

    /** The robot system instance containing other subsystems. */
    private final RobotSystem robot;

    private final HoodSubsystem hood;

    private final IntakeSubsystem intake;

    private final GateSubsystem gate;

    private final DriveSubsystem drive;

    public enum ShootState {
        NOT_SHOOTING,
        WAITING_TO_SHOOT,
        SHOOTING,
    }

    private ShootState shootState = ShootState.NOT_SHOOTING;

    // Allows for autonomous shooting overrides the distance calculation
    private double customDistance = DEFAULT_SHOOT_DISTANCE;

    private  boolean useCustomDistance = false;

    /**
     * Constructs a new DmShooterSubsystem.
     *
     * @param map     The hardware map for accessing robot components.
     * @param robot   The main robot system instance.
     */
    public ShooterSubsystem(HardwareMap map, RobotSystem robot) {
        this.robot = robot;
        voltageSensor = map.voltageSensor.iterator().next();
        hood = new HoodSubsystem(map);
        shooterMotor1 = map.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = map.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = robot.getIntake();
        gate = robot.getGateSubsystem();
        drive = robot.getDrive();
        rangeToRpm = Trajectories.createShooterTable();
    }

    /**
     * Periodic update for the subsystem. Handles PID control and state transitions.
     */
    @Override
    public void periodic() {

        // Update the distance
        updateDistance();

        // if we are shooting or waiting to shoot
        if (isActiveShot()) {

            // Do not run the intake/transfer until the shooter is ready
            if (shootState == ShootState.WAITING_TO_SHOOT) {
                setFeeding(false);
            }

            // We've hit target RPM, let them fly
            if (isShootReady() || shootState == ShootState.SHOOTING) {
                shootState = ShootState.SHOOTING;

//                if (drive.isAutoAim()) {
//                    drive.setAutoAim(false);
//                }

                gate.open();
                // wait for the gate to open before feeding
                if (gate.isOpen()) {
                    setFeeding(true);
                }
            } else {
                shootState = ShootState.WAITING_TO_SHOOT;

//                if (!drive.isAutoAim()) {
//                    drive.setAutoAim(true);
//                }

                // only update the hood and RPM while we are waiting to shoot
                // this keeps jitter from happening when we are shooting
                hood.setHoodAngle(lastDistance);
                setRpmFromRange(lastDistance);
                setFeeding(false);
            }

            // Apply the power to the motors based on PIDF and battery voltage
            setNormalizedPower();

        } else {
            setFloatPower(floatPower);
        }
    }

    /**
     * Updates the distance to the goal based on the camera's results, the robot's pose, or a custom distance.
     */
    private void updateDistance() {

        CameraSubsystem camera = robot.getCamera();
        LLResult result = robot.getCamera().getLastResult();

        if (boostRpm != 0) {
            telemetry.addData("Boost RPM", "%.2f", boostRpm);
        }

        double lastBotPoseDistance = Targets.getDistanceToTarget(robot.getDrive().getPose());
        double lastCameraDistance = camera.getValidCamDistance(result);
        String keyName;
        // Check if we are using custom distance
        if (customDistance != 0 && useCustomDistance) {
            lastDistance = customDistance;
            keyName = "Custom Distance";
        } else if (lastCameraDistance > 0) {
            // Only use camera distance if it is good
            lastDistance = lastCameraDistance;
            keyName = "Camera Distance";
        } else if (PoseTracker.isPoseValid()) {
            // Use robot pose when not getting good camera data
            lastDistance = lastBotPoseDistance;
            keyName = "Bot Pose Distance";
        } else if (result != null && result.isValid()){
            // Last ditch effort to get a valid angle on target by using any camera result
            lastDistance = CameraSubsystem.calcGoalDistanceByTa(result);
            keyName = "Weak Camera Distance";
        }else {
            lastDistance = DEFAULT_SHOOT_DISTANCE;
            keyName = "Default Distance!";
        }

        // TODO: remove most of these for competition
        telemetry.addData(keyName, "%.2f", lastDistance);

        if (lastBotPoseDistance > 0 && lastCameraDistance > 0)
            telemetry.addData("Delta Distance", "Bot %.2f -  Cam %.2f = %.2f",
                    lastBotPoseDistance, lastCameraDistance, lastBotPoseDistance - lastCameraDistance);
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
     * Applies the normalized power to the shooter motors.
     */
    private void setNormalizedPower() {
        updateVoltageCompensation();
        setPower(shooterPid.calculate(getCurrentRpm()));
    }

    /**
     * Updates the voltage compensation kF based on the current battery voltage.
     */
    private void updateVoltageCompensation() {
        double currentVoltage = getBatteryVoltage();
        if (currentVoltage > 8.0 && currentVoltage < NOMINAL_VOLTAGE) {
            double voltageDelta = (NOMINAL_VOLTAGE / currentVoltage) - 1.0;
            double compensatedKf = baseKf * (1.0 + (voltageDelta * KF_SCALER));
            shooterPid.setF(compensatedKf);
        } else {
            shooterPid.setF(baseKf);
        }
    }


    public void setUseCustomDistance(boolean useIt) {
        this.useCustomDistance = useIt;
    }

    public HoodSubsystem getHood(){
        return hood;
    }

    private void setFeeding(boolean feed) {
        intake.setAutoIntake(feed);
    }

    public void adjustCustomDistance(double increment) {
        this.customDistance += increment;
    }

    public void resetCustomDistance() {
        this.customDistance = DEFAULT_SHOOT_DISTANCE;
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

        double curMs = shootTimer.milliseconds();

        // Ready if RPM is within tolerance
        double currentRpm = getCurrentRpm();
        return (currentRpm - targetRpm >= VELOCITY_TOLERANCE_LOW) && (currentRpm - targetRpm <= VELOCITY_TOLERANCE_HIGH)
                //|| (curMs > MAX_SHOOT_TIME_MS)
                ;
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
       shooterPid.setPIDF(kp, ki, kd, kf);
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
        targetRpm = 0;
//        if (drive.isAutoAim()) {
//            drive.setAutoAim(false);
//        }

        useCustomDistance = false;
        shooterPid.reset();
        setFloat();
        gate.close();
        setFeeding(false);
    }

    public void setFloat() {
        shooterMotor1.setPower(floatPower);
        shooterMotor2.setPower(floatPower);
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
        useCustomDistance = distance > 0;
    }

    /**
     * Gets the set custom distance value that would be applied when enabled
     * @return the distance in inches
     */
    public double getCustomDistance() {
        return customDistance;
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

        targetRpm = rpm;
        shooterPid.setSetPoint(rpm);
    }

    public void setPower (double power) {
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    /**
     * Sets the idle (float) power applied when not shooting.
     *
     * @param power The power level (-1.0 to 1.0).
     */
    public void setFloatPower(double power) {
        floatPower = power;
        if (isActiveShot()) {
            return;
        }

        setFloat();
    }

    public void toggleFloatPower(){

        floatPower = floatPower > 0 ? 0 : FLOAT_DEFAULT_POWER;
        if (isActiveShot()){
            return;
        }

        setFloat();
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
     * Gets the motor velocity in ticks per second.
     * uses redundancy in case an encoder wire fails.
     *
     * @return The motor velocity.
     */
    private double getVelocity() {
        double v1 = shooterMotor1.getVelocity();
        double v2 = shooterMotor2.getVelocity();
        if (v1 > 0 && v2 > 0) {
            return (v1 + v2) / 2;
        }

        return Math.max(v1, v2);
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

    public DcMotorEx getShooterMotor1() {
        return shooterMotor1;
    }

    public DcMotorEx getShooterMotor2() {
        return shooterMotor2;
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
     * Command to resets the boost RPM.
     */
    public Command resetBoostRpmCommand() {
        return runOnce(this::resetBoostRpm);
    }

    /**
     * Returns a command that starts the shooting sequence.
     * with override distance
     */
    public Command startShootCmd(double overrideDistance) {
        return runOnce(() -> {
            if (overrideDistance != 0) {
                this.setDistance(overrideDistance);
            } else {
                useCustomDistance = false;
            }
            setStartShoot(true);

        });
    }

    /**
     * Returns a command that starts the shooting sequence.
     */
    public Command startShootCmd() {
        return runOnce(() -> setStartShoot(true));
    }


    /**
     * Returns a command that stops the shooting sequence.
     */
    public Command stopShootCmd() {
        return runOnce(() -> setStartShoot(false));
    }

    /**
     * Returns a command that adjusts the boost RPM.
     *
     * @param amount The amount to adjust.
     */
    public Command adjustBoostRpmCmd(double amount) {
        return runOnce(() -> adjustBoostRpm(amount));
    }

    /**
     * Returns a command that waits for shooter spinup and then shoots for given duration.
     *
     * @param durationMs The duration in milliseconds to wait after becoming ready.
     */
    public Command shootWhenReadyCmd(long durationMs) {
        return shootWhenReadyCmd(durationMs, 0);
    }

    /**
     * Returns a command that waits for shooter spinup and then shoots for given duration and distance
     * @param durationMs The duration in milliseconds to wait after becoming ready.
     * @param distanceOverride Override the distance to shoot to the goal good for not needing to calc distance
     */
    public Command shootWhenReadyCmd(long durationMs, double distanceOverride) {
        return startShootCmd(distanceOverride)
                .andThen(new WaitUntilCommand(this::isShootReady))
                .andThen(new WaitCommand(durationMs))
                .andThen(stopShootCmd());
    }

    /**
     * Puts the shooter motors in to float (AKA pre-spin)
     */
    public Command startFloatCmd() {
        return runOnce(()-> setFloatPower(FLOAT_DEFAULT_POWER));
    }

    /**
     * Turns off float mode
     */
    public Command stopFloatCmd() {
        return runOnce(()-> setFloatPower(0));
    }

    public void setUnjam(boolean run) {
        if (run) {
            shooterMotor1.setPower(1);
            shooterMotor2.setPower(1);
        }
        else {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
        }

    }


}