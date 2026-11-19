package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.tools.FieldConstants.BLUE_AUDIENCE_POSE;
import static org.firstinspires.ftc.teamcode.tools.FieldConstants.BLUE_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.tools.FieldConstants.RED_AUDIENCE_POSE;
import static org.firstinspires.ftc.teamcode.tools.FieldConstants.RED_SCORE_POSE;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.tools.PoseTracker;
import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.firstinspires.ftc.teamcode.tools.Targets;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.function.DoubleSupplier;

public class DriveSubsystem extends StealthSubsystem
{
    private final Follower follower;
    private final Telemetry telemetry = StealthOpMode.telemetry;
    private final CameraSubsystem camera;
    private DoubleSupplier forward = () -> 0;
    private DoubleSupplier strafe = () -> 0;
    private DoubleSupplier rotate = () -> 0;
    private boolean robotCentric = false;
    private boolean hold = true;
    private static final double SLOW_POWER = 0.3;
    private static final double MAX_POWER = 1.0;
    private boolean slowMo = false;
    private boolean autoAim = false;
    private boolean isTeleOp = true;

    private final PIDFController headingController = new PIDFController(
            0.02,
            0.0001,
            0.0015,
            0.0
    );

    private double kiNear = 0.11;
    private double kiFar = 0.0001;
    private boolean kiSet = false;

    /** The maximum rotational power applied during auto-aim. */
    public static final double MAX_ROTATION_POWER = 0.98;
    public static double AUTO_AIM_TOLERANCE = 1.0;
    public static double AUTO_AIM_CLOSE_TOLERANCE = 4.0;

    private double lastTxDelta = 100;


    public DriveSubsystem(HardwareMap map, RobotSystem robot){

        this.follower = Constants.create(map);
        this.camera = robot.getCamera();

        if (PoseTracker.isPoseValid()) {
            follower.setPose(PoseTracker.getPose());
        }else {
            follower.setPose(Alliance.isBlue() ? BLUE_AUDIENCE_POSE : RED_AUDIENCE_POSE);
        }

        setTeleOp(false);
    }

    /**
     * Sets the gamepad inputs for the drive subsystem.
     * @param driver The gamepad to use for input.
     */
    public void setDriver(GamepadEx driver) {
        forward = driver::getLeftY;
        strafe =  () -> -driver.getLeftX();
        rotate = () -> -driver.getRightX();
        setTeleOp(true);
    }


    @Override
    public void periodic(){
        follower.update();

        if (isTeleOp || autoAim) {
            DrivePowers powers = calculateDrivePowers();
            drive(powers);
        }
    }


    /**
     * Calculates the drive powers based on the gamepad inputs and auto aim powers.
     * @return The drive powers to be applied to the robot.
     */
    private DrivePowers calculateDrivePowers() {
        double invert = Alliance.isBlue() && !robotCentric ? -1 : 1;
        double currentPower = isSlowMo() ? SLOW_POWER : MAX_POWER;
        double forwardPow = forward.getAsDouble() * currentPower * invert;
        double strafePow = strafe.getAsDouble() * currentPower * invert;
        double rotatePow = rotate.getAsDouble() * currentPower;

        if (autoAim) {
            double targetAngle = Targets.getAngleErrorToTarget(getPose());
            LLResult result = camera.getLastResult();

            // Only use camera if we have a good distance
            if (camera.getValidCamDistance(result) > 0) {
                lastTxDelta = camera.getAdjustedTx(result);
            } else {
                lastTxDelta = targetAngle;
            }

            rotatePow = getScaledTxOutput(lastTxDelta, AUTO_AIM_TOLERANCE);
        }

        return robotCentric ?
                new DrivePowers(forwardPow, strafePow, rotatePow) :
                ManualDrive.fieldCentric(forwardPow, strafePow, rotatePow, follower.pose().heading());
    }

    /**
     * Applies the drive powers to the robot.
     * @param powers The drive powers to be applied to the robot.
     */
    private void drive(DrivePowers powers) {

        if (hold) {
            ManualDrive.driveOrHold(follower, powers);
        } else {
            follower.manual(powers);
        }
    }

    /**
     * Gets the scaled Tx power for Auto-aim based on the current Tx delta.
     * @param txDelta The current Tx delta.
     * @param tolerance Tolerance in degrees to consider the robot aimed.
     * @return The scaled Tx power.
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
     * Mainly for testing purposes allows updating the aim PIDF
     * @param kp Proportional gain
     * @param kiFar Integral gain for error large
     * @param kiNear Integral gain for error small
     * @param kd Derivative gain
     * @param kf Feedforward gain
     */
    public void setAimController(double kp, double kiFar, double kiNear, double kd, double kf) {
        this.kiNear = kiNear;
        this.kiSet = false;
        headingController.setPIDF(kp, kiFar, kd, kf);
        headingController.reset();
    }

    /**
     * Configures robot for being driven by controller
     * @param teleOp true for teleop otherwise false
     */
    public void setTeleOp(boolean teleOp) {
        isTeleOp = teleOp;
    }

    /**
     * Gets the aim controller (for testing)
     * @return The aim controller
     */
    public PIDFController getAimController() {
        return headingController;
    }

    /**
     * Gets the current pose of the robot
     * @return The current pose of the robot
     */
    public Pose getPose() {
        return follower.pose();
    }

    /**
     * Sets the robot to auto aim
     * @param autoAim true for auto aim otherwise false
     */
    public void setAutoAim(boolean autoAim) {
        this.autoAim = autoAim;
        this.kiSet = false;
        headingController.reset();
        // Just in case the robot was still moving
        if (!autoAim){
            drive(new DrivePowers(0, 0, 0));
        }
    }

    /**
     * Gets if the robot is in auto aim mode
     * @return true if auto aim otherwise false
     */
    public boolean isAutoAim() {
        return autoAim;
    }

    /**
     * Toggles the robot's centric mode (Robot centric drive like RC car)
     */
    public void toggleRobotCentric(){
        robotCentric = !robotCentric;
    }

    /**
     * Gets if the robot is in centric mode
     * @return true if centric otherwise false
     */
    public boolean isRobotCentric() {
        return robotCentric;
    }

    /**
     * Gets if the robot is in slow motion mode
     * @return true if slow motion otherwise false
     */
    public boolean isSlowMo() {
        return slowMo;
    }

    /**
     * Toggles the robot's slow motion mode
     */
    public void toggleSlowMo() {
        slowMo = ! slowMo;
    }

    /**
     * Toggles Pedro position hold mode
     */
    public void toggleHoldMode() {
        hold = ! hold;
    }

    /**
     * Gets if the robot is in hold mode
     * @return true if hold otherwise false
     */
    public boolean isHoldMode() {
        return hold;
    }

    /**
     * Sets the robot to hold mode
     * @param hold true for hold otherwise false
     */
    public void setHoldMode(boolean hold) {
        this.hold = hold;
    }

    /**
     * Sets the robot to slow motion mode
     * @param pressed true for slow motion otherwise false
     */
    public void setSlowMo(boolean pressed) {
        slowMo = pressed;
    }

    /**
     * Checks if the robot is aimed
     * @return true if aimed otherwise false
     */
    public boolean isAimed(){
        return (Math.abs(lastTxDelta) < AUTO_AIM_TOLERANCE);
    }

    /**
     * Sets the robot to auto aim mode
     * @param autoAim true for auto aim otherwise false
     */
    public Command setAutoAimCmd(boolean autoAim) {
        return runOnce(() -> setAutoAim(autoAim));
    }

    /**
     * Sets the robot to auto aim mode with timeout
     * @param timeoutMs timeout in milliseconds
     */
    public Command aimAtTargetCmd(long timeoutMs) {
        return setAutoAimCmd(true)
                .andThen(new WaitUntilCommand(this::isAimed).withTimeout(timeoutMs))
                .andThen(setAutoAimCmd(false));
    }

    /**
     * Toggles slow motion mode
     */
    public Command toggleSlowMoCmd() {
        return runOnce(this::toggleSlowMo);
    }

    /**
     * Sets the robot to slow motion mode
     * @param slowMo true for slow motion otherwise false
     */
    public Command setSlowMoCmd(boolean slowMo) {
        return runOnce(() -> setSlowMo(slowMo));
    }

    /**
     * Toggles centric mode
     */
    public Command toggleRobotCentricCommand() {
        return runOnce(this::toggleRobotCentric);
    }

    /**
     * Sets the robot to centric mode
     */
    public Command toggleHoldModeCmd() {
        return runOnce(this::toggleHoldMode);
    }

    /**
     * Applies drive powers to the robot based on the provided values
     * @param forward forward direction power
     * @param strafe strafe direction power
     * @param rotate rotation power
     */
    public void drive(double forward, double strafe, double rotate){
        follower.manual(new com.pedropathing.drivetrain.DrivePowers(forward, strafe, rotate));
    }

    /**
     * Allows for direct access to the follower object
     * @return Pedro follower
     */
    public Follower getFollower(){
        return follower;
    }

    /**
     * Updates the robot's position based on the provided pose.
     * prevents the robot from moving due to hold mode
     * @param pose the new pose to update to
     */
    public void updatePosition(Pose pose) {
        boolean oldHold = isHoldMode();
        setHoldMode(false);
        follower.update();

        follower.setPose(pose);
        // This keeps the robot from moving during the update
        drive(0, 0, 0);
        follower.update();
        PoseTracker.setPose(pose);
        setHoldMode(oldHold);
    }

    /**
     * Resets the robots heading only and prevents movement of robot when hold is active
     */
    public Command resetHeadingCmd() {
        boolean oldHold = isHoldMode();
        return runOnce(()-> setHoldMode(false))
                .andThen(runOnce(follower::update))
                .andThen(runOnce(this::resetHeading))
                .andThen(runOnce(follower::update))
                .andThen(runOnce(() -> setHoldMode(oldHold)));
    }

    /**
     * Resets robot position to audience corner or hive center of the legs
     * turns off hold mode to prevent robot movement during reset
     * @param audience true for audience otherwise false
     */
    public Command resetPositionCmd(boolean audience) {
        boolean oldHold = isHoldMode();
        return runOnce(()-> setHoldMode(false))
                .andThen(runOnce(follower::update))
                .andThen(runOnce(() -> resetPosition(audience))
                .andThen(runOnce(follower::update))
                .andThen(runOnce(() -> setHoldMode(oldHold))))
                .andThen(runOnce(()-> PoseTracker.setPose(getPose())));
    }


    /**
     * Follows a path with no timeout and no hold
     * @param path Pedro path to follow
     */
    public Command followPathCmd(Path path){
        return followPathCmd(path, true);
    }

    /**
     * Follows a path with a timeout and no hold
     * @param path Pedro path to follow
     * @param timeout number of milliseconds to wait for path to finish
     * @return Command to follow path
     */
    public Command followPathCmd(Path path, long timeout){
        return followPathCmd(path, timeout,false);
    }
    /**
     * Follows a path with then optionally holds at the end point
     * @param path to follow path
     * @return Command to follow path
     * @param holdPoint true for hold at the end otherwise false
     */
    public Command followPathCmd(Path path, boolean holdPoint){
        return runOnce(() -> follower.follow(path))
                .andThen(new WaitUntilCommand(() -> !follower.following())
                        .whenFinished(() -> {
                            setHoldMode(holdPoint);
                            if (holdPoint) {
                                follower.hold(path.endPose());
                            }
                            setAutoAim(false);
                            follower.update();
                        }));
    }

    /**
     * Follows a path with a timeout and optionally holds at the end point
     * @param path to follow path
     * @param timeout number of milliseconds to wait for path to finish
     * @param holdPoint true for hold at the end otherwise false
     */
    public Command followPathCmd(Path path, long timeout, boolean holdPoint){
        return runOnce(() -> follower.follow(path))
                .andThen(new WaitUntilCommand(() -> !follower.following()).withTimeout(timeout))
                .whenFinished(() -> {
                    setHoldMode(holdPoint);
                    if (holdPoint) {
                        follower.hold(path.endPose());
                    }

                    setAutoAim(false);
                    follower.update();
                });
    }

    /**
     * Resets the robot to the scoring side corner wall or audience side corner wall
     * @param audience true for audience otherwise false
     */
    private void resetPosition(boolean audience) {
        Pose pose = audience ? (Alliance.isBlue() ? BLUE_AUDIENCE_POSE : RED_AUDIENCE_POSE) : (Alliance.isBlue() ? BLUE_SCORE_POSE : RED_SCORE_POSE);
        follower.setPose(pose);
    }

    /**
     * Resets the robots heading based on alliance
     */
    private void resetHeading() {
        follower.setHeading(Alliance.isBlue() ? FieldConstants.ROBOT_BLUE_HEADING : FieldConstants.ROBOT_RED_HEADING);
    }

}