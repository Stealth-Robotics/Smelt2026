package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.tools.AutoToTeleOpStorage;
import org.firstinspires.ftc.teamcode.tools.FieldConstants;
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

    // Rests to the Audience side Blue wall intake facing away from driver
    private static final Pose bluePose = new Pose(
            FieldConstants.FIELD_X_MAX - FieldConstants.ROBOT_CENTER_X,
            FieldConstants.ROBOT_CENTER_Y,
            FieldConstants.ROBOT_BLUE_HEADING);

    // Rests to the Audience side Red wall intake facing away from driver
    // TODO: RED side may be set to far red for testing check if this is correct FieldConstants.FIELD_Y_MAX - FieldConstants.ROBOT_CENTER_Y makes it far red
    private static final Pose redPose = new Pose(
            FieldConstants.ROBOT_CENTER_X,
            FieldConstants.FIELD_Y_MAX -
                    FieldConstants.ROBOT_CENTER_Y,
            FieldConstants.ROBOT_RED_HEADING);

    public DriveSubsystem(HardwareMap map, RobotSystem robot){

        this.follower = Constants.create(map);
        this.camera = robot.getCamera();

        if (AutoToTeleOpStorage.isPoseValid()) {
            follower.setPose(AutoToTeleOpStorage.getPose());
        }else {
            follower.setPose(Alliance.isBlue() ? bluePose : redPose);
        }

        //follower.localizer.reset();
        setTeleOp(false);
    }

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

        return robotCentric ?
                new DrivePowers(forwardPow, strafePow, rotatePow) :
                ManualDrive.fieldCentric(forwardPow, strafePow, rotatePow, follower.pose().heading());
    }

    private void drive(DrivePowers powers) {

        if (hold) {
            ManualDrive.driveOrHold(follower, powers, 0.05, 0.1);
        } else {
            follower.manual(powers);
        }
    }

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

    public void setAimController(double kp, double kiFar, double kiNear, double kd, double kf) {
        this.kiNear = kiNear;
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
        // Just in case the robot was still moving
        if (!autoAim){
            drive(new DrivePowers(0, 0, 0));
        }
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
        return new InstantCommand(() -> setAutoAim(autoAim), this);
    }

    public Command aimAtTargetCommand(long timeoutMs) {
        return setAutoAimCommand(true)
                .andThen(new WaitUntilCommand(this::isAimed).withTimeout(timeoutMs))
                .andThen(setAutoAimCommand(false));
    }

    public Command toggleSlowMoCommand() {
        return new InstantCommand(this::toggleSlowMo, this);
    }

    public Command setSlowMoCommand(boolean slowMo) {
        return new InstantCommand(() -> setSlowMo(slowMo), this);
    }

    public Command toggleRobotCentricCommand() {
        return new InstantCommand(this::toggleRobotCentric, this);
    }

    public Command toggleHoldModeCommand() {
        return new InstantCommand(this::toggleHoldMode, this);
    }

    public void Drive(double forward, double strafe, double rotate){
        follower.manual(new com.pedropathing.drivetrain.DrivePowers(forward, strafe, rotate));
    }

    public Follower getFollower(){
        return follower;
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void resetOdometry(boolean resetPose) {
        Pose basePose = resetPose ? (Alliance.isBlue() ? bluePose : redPose) : follower.pose();

        double heading = Alliance.isBlue() ? FieldConstants.ROBOT_BLUE_HEADING : FieldConstants.ROBOT_RED_HEADING;
        follower.setPose(new Pose(basePose.x(), basePose.y(), heading));
    }

    public Command resetOdometryCommand(boolean resetPose) {
        return runOnce(() -> resetOdometry(resetPose));
    }

    public Command resetOdometryCommand() {
        return resetOdometryCommand(false);
    }

    /**
     * Follows a path with no timeout and no hold
     * @param path Pedro path to follow
     * @return Command to follow path
     */
    public Command followPathCommand(Path path){
        return followPathCommand(path, false);
    }

    /**
     * Follows a path with then optionally holds at the end point
     * @param path to follow path
     * @return Command to follow path
     * @param holdPoint true for hold at the end otherwise false
     */
    public Command followPathCommand(Path path, boolean holdPoint){
        return runOnce(() -> follower.follow(path))
                .andThen(new WaitUntilCommand(follower::atParametricEnd)
                        .whenFinished(() -> { if (holdPoint) follower.hold(path.endPose()); }));
    }

    /**
     * Follows a path with a timeout and optionally holds at the end point
     * @param path to follow path
     * @param timeout number of milliseconds to wait for path to finish
     * @param holdPoint true for hold at the end otherwise false
     */
    public Command followPathCommand(Path path, long timeout, boolean holdPoint){
        long endtime = System.currentTimeMillis() + timeout;
        return runOnce(() -> follower.follow(path))
                .andThen(new WaitUntilCommand(() -> (follower.atParametricEnd() || System.currentTimeMillis() > endtime)))
                .whenFinished(() -> { if (holdPoint) follower.hold(path.endPose()); });
    }
}