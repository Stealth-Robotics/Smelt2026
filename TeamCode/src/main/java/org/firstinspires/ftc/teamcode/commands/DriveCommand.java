package org.firstinspires.ftc.teamcode.commands;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.firstinspires.ftc.teamcode.tools.Pipeline;
import org.stealthrobotics.library.Alliance;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotate;
    private boolean robotCentric = false;
    private boolean hold = false;
    private static final double SLOW_POWER = 0.3;
    private static final double MAX_POWER = 1.0;
    private boolean slowMo = false;

    private boolean autoAim = false;

    private boolean isTeleOp = true;

    private final PIDFController headingController = new PIDFController(
            0.02,
            0.0001,
            0.0015,
            0.0);

    private double kiNear = 0.12;
    private double kiFar = 0.0001;
    private boolean kiSet = false;

    /** The maximum rotational power applied during auto-aim. */
    public static final double MAX_ROTATION_POWER = 0.98;
    public static double AUTO_AIM_TOLERANCE = 0.5;

    public static double AUTO_AIM_CLOSE_TOLERANCE = 4.0;

    private double lastTxDelta = 100;

    private final CameraSubsystem camera;

    private final Follower follower;

    public DriveCommand(
            RobotSystem robot,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotate){
        this.driveSubsystem = robot.getDriveSubsystem();
        this.camera = robot.getCamera();
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
        this.follower = driveSubsystem.getFollower();
    }

    public DriveCommand(RobotSystem robot){
        this(robot, () -> 0, () -> 0, () -> 0);
        isTeleOp = false;
    }

    @Override
    public void execute() {
        updateCameraPipeline();
        telemetry.addData("Auto Aim", autoAim);
        if (isTeleOp || autoAim) {
            DrivePowers powers = calculateDrivePowers();
            applyDrivePowers(powers);
        }
    }

    private void updateCameraPipeline() {
        Pose cuPose = follower.pose();
        camera.updateRobotOrientation(cuPose.heading());

        boolean isNear = cuPose.y() < FieldConstants.FIELD_Y_MAX / 2.0;
        Pipeline pipeline = Alliance.isBlue()
                ? (isNear ? Pipeline.BLUE_NEAR : Pipeline.BLUE_FAR)
                : (isNear ? Pipeline.RED_NEAR : Pipeline.RED_FAR);

        camera.setPipeline(pipeline);
    }

    private DrivePowers calculateDrivePowers() {
        double invert = Alliance.isBlue() && !robotCentric ? -1 : 1;
        double currentPower = isSlowMo() ? SLOW_POWER : MAX_POWER;
        double forwardPow = forward.getAsDouble() * currentPower * invert;
        double strafePow = strafe.getAsDouble() * currentPower * invert;
        double rotatePow = rotate.getAsDouble() * currentPower;

        LLResult result = camera.getLastResult();
        if (result != null && result.isValid()) {
            double txDelta = camera.getAdjustedTx(result);
            lastTxDelta = txDelta;
            if (autoAim) {
                rotatePow = getScaledTxOutput(txDelta, AUTO_AIM_TOLERANCE);
            }
        } else {
            lastTxDelta = 100;
        }

        telemetry.addData("Last Tx Delta", lastTxDelta);

        return robotCentric ?
                new DrivePowers(forwardPow, strafePow, rotatePow) :
                ManualDrive.fieldCentric(forwardPow, strafePow, rotatePow, follower.pose().heading());
    }

    private void applyDrivePowers(DrivePowers powers) {
        telemetry.addData("Drive Powers", powers);
        if (hold) {
            ManualDrive.driveOrHold(follower, powers, 0.05, 0.1);
        } else {
            follower.manual(powers);
        }
    }

    /**
     * Calculates the required rotational power to correct a given heading error (txDelta)
     * using a PIDF controller. The output is clamped to a maximum power.
     *
     * @param txDelta   The horizontal error from the target (e.g., Limelight's 'tx').
     * @param tolerance The error margin within which no correction is applied.
     * @return The calculated rotational power, or 0 if within tolerance.
     */
    private double getScaledTxOutput(double txDelta, double tolerance) {

        if (Math.abs(txDelta) < tolerance) {
            headingController.clearTotalError();
            return 0;
        }

        if (Math.abs(txDelta) < AUTO_AIM_CLOSE_TOLERANCE) {

                if (!kiSet) {
                    kiSet = true;
                    headingController.clearTotalError();
                }

                headingController.setI(kiNear);

        } else {
            if (kiSet) {
                kiSet = false;
                headingController.clearTotalError();
            }

            headingController.setI(kiFar);
        }

        // Setpoint is 0 degrees offset; calculate error using current txDelta measurement.
        headingController.setSetPoint(0);
        double pidOutput = headingController.calculate(txDelta);
        return Math.clamp(pidOutput, -MAX_ROTATION_POWER, MAX_ROTATION_POWER);
    }

    /**
     * Overrides the aim PIDF controller for testing purposes.
     * @param kp Proportional gain
     * @param kiFar Far away from target Integral gain
     * @param kiNear Near target Integral gain
     * @param kd Derivative gain
     * @param kf Feedforward gain
     */
    public void setAimController(double kp, double kiFar, double kiNear, double kd, double kf) {
        this.kiNear = kiNear;
        this.kiFar = kiFar;
        this.kiSet = false;
        headingController.setPIDF(kp, kiFar, kd, kf);
        headingController.reset();
    }

    public void setTeleOp(boolean teleOp) {
        isTeleOp = teleOp;
    }

    public PIDFController getAimController() {
        return headingController;
    }

    public void setAutoAim(boolean autoAim) {
        this.autoAim = autoAim;
        this.kiSet = false;
        headingController.reset();
    }

    public void toggleRobotCentric(){
        robotCentric = !robotCentric;
    }

    public boolean isRobotCentric() {
        return robotCentric;
    }

    public boolean isSlowMo() {
        return slowMo;
    }
    
    public void toggleSlowMo() {
        slowMo = ! slowMo;
    }

    public void toggleHoldMode() {
        hold = ! hold;
    }

    public boolean isHoldMode() {
        return hold;
    }

    public void setSlowMo(boolean pressed) {
        slowMo = pressed;
    }

    public boolean isAimed(){
        return (Math.abs(lastTxDelta) < AUTO_AIM_TOLERANCE);
    }

    public Command setAutoAimCommand(boolean autoAim) {
        return new InstantCommand(() -> setAutoAim(autoAim));
    }

    public Command aimAtTargetCommand(long timeoutMs) {
        return setAutoAimCommand(true)
                .andThen(new WaitUntilCommand(this::isAimed).withTimeout(timeoutMs))
                .andThen(setAutoAimCommand(false));
    }

    public Command toggleSlowMoCommand() {
        return new InstantCommand(this::toggleSlowMo);
    }

    public Command setSlowMoCommand(boolean slowMo) {
        return new InstantCommand(() -> setSlowMo(slowMo));
    }

    public Command toggleRobotCentricCommand() {
        return new InstantCommand(this::toggleRobotCentric);
    }

    public Command toggleHoldModeCommand() {
        return new InstantCommand(this::toggleHoldMode);
    }

    /**
     * @deprecated Path following commands belong on {@link DriveSubsystem}. Use
     * {@code driveSubsystem.followPathCommand(path)} directly.
     */
    @Deprecated
    public Command followPathCommand(Path path){
        return driveSubsystem.followPathCommand(path);
    }

    /**
     * @deprecated Path following commands belong on {@link DriveSubsystem}. Use
     * {@code driveSubsystem.followPathCommand(path, holdPoint)} directly.
     */
    @Deprecated
    public Command followPathCommand(Path path, boolean holdPoint){
        return driveSubsystem.followPathCommand(path, holdPoint);
    }

    /**
     * @deprecated Path following commands belong on {@link DriveSubsystem}. Use
     * {@code driveSubsystem.followPathCommand(path, timeout, holdPoint)} directly.
     */
    @Deprecated
    public Command followPathCommand(Path path, long timeout, boolean holdPoint){
        return driveSubsystem.followPathCommand(path, timeout, holdPoint);
    }

}
