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
import org.firstinspires.ftc.teamcode.tools.AutoToTeleOpStorage;
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

    private final PIDFController fieldHeadingController = new PIDFController(
            0.02,
            0.0001,
            0.0015,
            0.0
    );

    private double kiNear = 0.12;
    private double kiFar = 0.0001;
    private boolean kiSet = false;

    /** The maximum rotational power applied during auto-aim. */
    public static final double MAX_ROTATION_POWER = 0.98;
    public static double AUTO_AIM_TOLERANCE = 2.0;
    public static double AUTO_AIM_CLOSE_TOLERANCE = 4.0;
    public static double AUTO_AIM_BACKUP_TOLERANCE = 5.0; // in degrees

    private double lastTxDelta = 100;


    public DriveSubsystem(HardwareMap map, RobotSystem robot){

        this.follower = Constants.create(map);
        this.camera = robot.getCamera();

        if (AutoToTeleOpStorage.isPoseValid()) {
            follower.setPose(AutoToTeleOpStorage.getPose());
        }else {
            follower.setPose(Alliance.isBlue() ? BLUE_AUDIENCE_POSE : RED_AUDIENCE_POSE);
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

        lastTxDelta = Targets.getAngleErrorToTarget( getPose());

        LLResult result = camera.getLastResult();
        if (result != null && result.isValid()) {
            lastTxDelta = camera.getAdjustedTx(result);
        }

        if (autoAim) {
            rotatePow = getScaledTxOutput(lastTxDelta, AUTO_AIM_TOLERANCE);
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

    private double getScaledPoseOutput(double headingError, double tolerance) {
        if(Math.abs(headingError) < tolerance) {
            fieldHeadingController.clearTotalError();
            return 0;
        }
        fieldHeadingController.setSetPoint(0);
        double pidOutput = fieldHeadingController.calculate(headingError);
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

    public Pose getPose() {
        return follower.pose();
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

    public void setHoldMode(boolean hold) {
        this.hold = hold;
    }

    public void setSlowMo(boolean pressed) {
        slowMo = pressed;
    }

    public boolean isAimed(){
        return (Math.abs(lastTxDelta) < AUTO_AIM_TOLERANCE);
    }

    public Command setAutoAimCmd(boolean autoAim) {
        return runOnce(() -> setAutoAim(autoAim));
    }

    public Command aimAtTargetCmd(long timeoutMs) {
        return setAutoAimCmd(true)
                .andThen(new WaitUntilCommand(this::isAimed).withTimeout(timeoutMs))
                .andThen(setAutoAimCmd(false));
    }

    public Command toggleSlowMoCmd() {
        return runOnce(this::toggleSlowMo);
    }

    public Command setSlowMoCmd(boolean slowMo) {
        return runOnce(() -> setSlowMo(slowMo));
    }

    public Command toggleRobotCentricCommand() {
        return runOnce(this::toggleRobotCentric);
    }

    public Command toggleHoldModeCmd() {
        return runOnce(this::toggleHoldMode);
    }

    public void Drive(double forward, double strafe, double rotate){
        follower.manual(new com.pedropathing.drivetrain.DrivePowers(forward, strafe, rotate));
    }

    public Follower getFollower(){
        return follower;
    }

    /**
     * Resets the robots heading only and prevents movement of robot when hold is active
     * @return Command to reset heading
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
     * Resets robot position to scoring side corner wall or audience side corner wall
     * turns off hold mode to prevent robot movement during reset
     * @param audience true for audience otherwise false
     * @return Command to reset position
     */
    public Command resetPositionCmd(boolean audience) {
        boolean oldHold = isHoldMode();
        return runOnce(()-> setHoldMode(false))
                .andThen(runOnce(follower::update))
                .andThen(runOnce(() -> resetPosition(audience))
                .andThen(runOnce(follower::update))
                .andThen(runOnce(() -> setHoldMode(oldHold))));
    }


    /**
     * Follows a path with no timeout and no hold
     * @param path Pedro path to follow
     * @return Command to follow path
     */
    public Command followPathCmd(Path path){
        return followPathCmd(path, true);
    }

    public Command setStartPoseCmd(Pose pose){
        return runOnce(() -> follower.setPose(pose));
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
                        .whenFinished(() -> { if (holdPoint) follower.hold(path.endPose()); }));
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
                    if (holdPoint) follower.hold(path.endPose());
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