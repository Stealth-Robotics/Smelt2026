package org.firstinspires.ftc.teamcode.commands;

import static org.stealthrobotics.library.Commands.runOnce;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
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
    private static final double slowPower = 0.3;
    private static final double maxPower = 1.0;
    private boolean slowMo = false;

    private boolean autoAim = false;

    private final PIDFController headingController = new PIDFController(
            0.0012,
            0.08,
            0,
            0.005);

    private double kiNear = 0;
    private boolean kiSet = false;

    /** The maximum rotational power applied during auto-aim. */
    public static final double MAX_ROTATION_POWER = 0.98;
    public static double AUTO_AIM_TOLERANCE = 0.1;

    public static double AUTO_AIM_CLOSE_TOLERANCE = 3;

    private final CameraSubsystem camera;

    public final Follower follower;

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

    @Override
    public void execute(){
        Follower follower = driveSubsystem.getFollower();
        camera.updateRobotOrientation(follower.pose().heading());
        Pose cuPose = follower.pose();

        Pipeline pipeline;
        if (Alliance.isBlue()) {
            pipeline = (cuPose.y() < FieldConstants.FIELD_Y_MAX / 2)
                            ? Pipeline.BLUE_NEAR 
                            : Pipeline.BLUE_FAR;
        } else {
            pipeline = (cuPose.y() < FieldConstants.FIELD_Y_MAX / 2)
                            ? Pipeline.RED_NEAR 
                            : Pipeline.RED_FAR;
        }

        // setPipeline handles the case where the pipeline is already active
        camera.setPipeline(pipeline);

        double invert = Alliance.isBlue() && !robotCentric ? -1 : 1;
        double currentPower = isSlowMo() ? slowPower : maxPower;
        double forwardPow = forward.getAsDouble() * currentPower * invert;
        double strafePow = strafe.getAsDouble() * currentPower * invert;
        double rotatePow = rotate.getAsDouble() * currentPower;

        if (autoAim) {

            LLResult result = camera.getLastResult();
            if (result != null && result.isValid()) {
                double txDelta = camera.getAdjustedTx(result);
                txDelta = Math.round(txDelta * 1000) / 1000.0;
                rotatePow = getScaledTxOutput(txDelta, AUTO_AIM_TOLERANCE);
                telemetry.addData("Adjusted Tx", txDelta);
                telemetry.addData("Scaled Output", rotatePow);
            }
        }

        DrivePowers powers = robotCentric?
                new DrivePowers(forwardPow, strafePow, rotatePow):
                ManualDrive.fieldCentric(forwardPow, strafePow, rotatePow, follower.pose().heading());

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

                headingController.setI(kiNear);

        } else {
            headingController.setI(0);
            headingController.clearTotalError();

        }

        // Set the PID controller's target to Tx
        headingController.setSetPoint(txDelta);
        // Update the controller with the Error of 0. Yes this feels backwards.
        double pidOutput = -headingController.calculate(0);
        return  Math.clamp(pidOutput, -MAX_ROTATION_POWER, MAX_ROTATION_POWER);
    }

    public void setAimController(double kp, double ki, double kd, double kf) {
        kiNear = ki;
        headingController.setPIDF(kp, 0, kd, kf);
        headingController.reset();
    }

    public PIDFController getAimController() {
        return headingController;
    }

    public void setAutoAim(boolean autoAim) {
        this.autoAim = autoAim;
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

    /**
     * Follows a path with no timeout and no hold
     * @param path Pedro path to follow
     * @return Command
     */
    public Command followPathCommand(Path path){
        return followPathCommand(path, false);
    }

    /**
     * Follows a path with then optionally holds at the end point
     * @param path Path to follow
     * @param holdPoint true for hold at end otherwise false
     * @return Command
     */
    public Command followPathCommand(Path path, boolean holdPoint){
        return runOnce(()-> follower.follow(path))
                .andThen(new WaitUntilCommand(()->
                        !follower.isBusy())
                        .whenFinished(
                        ()-> { if (holdPoint) follower.hold(path.endPose()); }));
    }

    /**
     * Follows a path with a timeout and optionally holds at the end point
     * @param path Path to follow
     * @param timeout number of milliseconds to wait for path to finish
     * @param holdPoint true for hold at end otherwise false
     * @return Command
     */
    public Command followPathCommand(Path path, long timeout, boolean holdPoint){
        long endtime = System.currentTimeMillis() + timeout;
        return runOnce(()-> follower.follow(path))
                .andThen(new WaitUntilCommand(()-> (!follower.isBusy() || System.currentTimeMillis() > endtime)))
                .whenFinished(()-> { if (holdPoint) follower.hold(path.endPose()); });
    }

}
