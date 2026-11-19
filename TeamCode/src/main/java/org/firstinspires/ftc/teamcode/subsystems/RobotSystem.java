package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.firstinspires.ftc.teamcode.tools.Pipeline;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;


/**
 * The primary subsystem manager for the robot.
 * This class initializes all hardware subsystems and manages gamepad bindings for driver and operator.
 */
public class RobotSystem extends StealthSubsystem {
    /** Gamepad for the driver */
    private final GamepadEx driver;
    /** Gamepad for the operator */
    private final GamepadEx operator;
    private final IntakeSubsystem intakeSubsystem;
    private final DmShooterSubsystem shooter;
    private final GateSubsystem gateSubsystem;
    private final HarvesterSubsystem harvesterSubsystem;
    private final DriveSubsystem drive;
    private final CameraSubsystem camera;

    /** Telemetry for robot status information */
    private final Telemetry telemetry = StealthOpMode.telemetry;

    private static final ElapsedTime poseTimer = new ElapsedTime();
    private static final double POSE_TIMEOUT_MS = 5000;
    private boolean updatePose = false;
    /**
     * Initializes the robot system with the provided hardware map and gamepads.
     *
     * @param hardwareMap The hardware map provided by the OpMode.
     * @param driver The gamepad used by the driver.
     * @param operator The gamepad used by the operator.
     */
    public RobotSystem(HardwareMap hardwareMap, GamepadEx driver, GamepadEx operator) {
        camera = new CameraSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        gateSubsystem = new GateSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, this);
        shooter = new DmShooterSubsystem(hardwareMap, this);
        harvesterSubsystem = new HarvesterSubsystem(hardwareMap);
        this.driver = driver;
        this.operator = operator;
    }


    /**
     * Configures gamepad bindings for both the driver and the operator.
     */
    public void configureBindings() {
        configDriverBindings();
        configOperatorBindings();
    }

    /**
     * Driver bindings
     */
    public void configDriverBindings() {
        // --- Driver Controls ---
        drive.setDriver(driver);
        setUpdatePose(true);
        // intake is analog left reverses and right intakes
        intakeSubsystem.setDefaultCommand(intakeSubsystem.runIntakeCommand(
                () -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

        // Reset the heading only
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(drive.resetHeadingCmd());

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(drive.toggleHoldModeCmd());

        // Reset the position to the Scoring side corner wall
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(drive.resetPositionCmd(false));
        // Reset the position to the audience side corner wall
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(drive.resetPositionCmd(true) );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(drive.setSlowMoCmd(true))
                .whenReleased(drive.setSlowMoCmd(false));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(drive.setAutoAimCmd(true))
                .whenReleased(drive.setAutoAimCmd(false));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(()-> shooter.setStartShoot(true))
                .whenReleased(()-> shooter.setStartShoot(false));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(shooter::toggleFloatPower);

        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(()->shooter.setUseCustomDistance(true));

    }

    /**
     * Operator bindings
     */
    public void configOperatorBindings() {
        // --- Operator Controls ---

//        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(this::togglePositionUpdates);

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                ()-> shooter.adjustCustomDistance(0.5));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                ()-> shooter.adjustCustomDistance(-0.5));

        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(()->drive.setAutoAim(true)).whenReleased(() -> drive.setAutoAim(false));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(unjamGateCmd(true))
                .whenReleased(unjamGateCmd(false));

        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(shooter::toggleFloatPower);

        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(harvesterSubsystem.harvToggleCmd());

        operator.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(shooter::resetCustomDistance);

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(shooter.adjustBoostRpmCmd(25));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(shooter.adjustBoostRpmCmd(-25));
    }

    @Override
    public void periodic(){
        updateCameraPipeline();
        updateDrivePosition();

//        telemetry.addData("Alliance: ", Alliance.get());
//        telemetry.addData("Slow Motion", drive.isSlowMo());
//        telemetry.addData("Robot Centric", drive.isRobotCentric());
//        telemetry.addData("Hold Mode", drive.isHoldMode());


        telemetry.addData("Position Updates", updatePose);

        // TODO: remove for COMP
        if (updatePose) {
            telemetry.addData("Last Pose update", poseTimer.seconds());
        }

//        Pose pose = drive.getPose();
//        telemetry.addData("Drive Pose", "Tx: %.2f, Ty: %.2f, Z: %.2f", pose.x(), pose.y(),  Math.toDegrees(pose.heading()));
//        Velocity velocity = drive.getFollower().velocity();
//        double linearSpeed = Math.hypot(velocity.vx, velocity.vy); // inches per second
//        double angularSpeed = Math.abs(velocity.omega);            // radians per second
//        telemetry.addData("Velocity", "Linear: %.2f, Angular: %.2f", linearSpeed, Math.toDegrees(angularSpeed));
//
//        LLResult result = camera.getLastResult();
//        if (result != null && result.isValid()){
//            pose = CameraSubsystem.toPedroPose(result.getBotpose());
//            telemetry.addData("Camera Pose: ", "X: %.2f, Y:%.2f, Z:%.2f", pose.x(),pose.y(), Math.toDegrees(pose.heading()));
//            pose = CameraSubsystem.toPedroPose(result.getBotpose_MT2());
//            telemetry.addData("Mega Tag 2", "X: %.2f, Y:%.2f, Z:%.2f", pose.x(),pose.y(), Math.toDegrees(pose.heading()));
//        }
    }

    /**
     * Enables or disables updating robot pose from AprilTags
     * @param update true for use tags otherwise false
     */
    public void setUpdatePose(boolean update) {
        this.updatePose = update;
    }

    public void togglePositionUpdates() {
        updatePose = !updatePose;
    }

    /**
     * Allows direct access to the gate servo
     * @return GateSubsystem
     */
    public GateSubsystem getGateSubsystem(){
        return gateSubsystem;
    }

    /**
     * Allows direct access to the HarvesterSubsystem
     * @return HarvesterSubsystem
     */
    public HarvesterSubsystem getHarvesterSubsystem(){
        return harvesterSubsystem;
    }

    /** @return The drive subsystem */
    public DriveSubsystem getDrive() {
        return drive;
    }

    /** @return The camera subsystem */
    public CameraSubsystem getCamera() {
        return camera;
    }

    /** @return The shooter subsystem */
    public DmShooterSubsystem getShooter() {
        return shooter;
    }

    /** @return The intake subsystem */
    public IntakeSubsystem getIntake() {
        return intakeSubsystem;
    }

    private Command unjamGateCmd(boolean unjam)  {
        if (unjam) {
            return new SequentialCommandGroup(
                    new InstantCommand(() -> intakeSubsystem.setAutoOuttake(true)),
                    new WaitCommand(500),
                    new InstantCommand(gateSubsystem::open),
                    new InstantCommand(() -> shooter.setUnjam(true)));
        }
        return new SequentialCommandGroup(
                new InstantCommand(gateSubsystem::close),
                new InstantCommand(() -> intakeSubsystem.setAutoOuttake(false)),
                new InstantCommand(() -> shooter.setUnjam(false)));
    }

    /**
     * Reads the Limelight 3D position and updates the robot's field position.
     */
    public void updateDrivePosition() {
        // First check if we should update
        if (!updatePose || poseTimer.milliseconds() < POSE_TIMEOUT_MS) {
            return;
        }

        // Check if the robot is moving too much
        Velocity velocity = drive.getFollower().velocity();
        double linearSpeed = Math.hypot(velocity.vx, velocity.vy); // inches per second
        double angularSpeed = Math.abs(velocity.omega);            // radians per second
        // TODO: determine thresholds for when the robot is considered "slow enough"
        final double maxLinearSpeed = 4.0;
        final double maxAngularSpeed = Math.toRadians(10);

        if (linearSpeed > maxLinearSpeed || angularSpeed > maxAngularSpeed) {
            return;
        }

        // finally check if we have a good enough vision measurement to update with
        LLResult result = camera.getLastResult();
        if (camera.isCamPoseValid(result)){
            assert result != null;
            // TODO: decide to use MT2 or MT1 to update position
            drive.updatePosition(CameraSubsystem.toPedroPose(result.getBotpose_MT2()));
            poseTimer.reset();
        }
    }

    /**
     * Updates the camera pipeline based on the robot's current pose.
     * Assumes the drive subsystem's follower is initialized.
     */
    public void updateCameraPipeline() {
        Pose cuPose = drive.getFollower().pose();
        camera.updateRobotOrientation(cuPose.heading());

        boolean isAudienceSide = cuPose.y() < FieldConstants.FIELD_Y_CENTER;
        Pipeline pipeline = Alliance.isBlue()
                ? (isAudienceSide ? Pipeline.BLUE_AUDIENCE : Pipeline.BLUE_SCORE)
                : (isAudienceSide ? Pipeline.RED_AUDIENCE : Pipeline.RED_SCORE);

        camera.setPipeline(pipeline);
    }
}