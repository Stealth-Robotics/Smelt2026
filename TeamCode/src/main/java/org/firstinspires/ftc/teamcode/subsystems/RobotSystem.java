package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private final DoubleShooterSubsystem shooter;
    private final GateSubsystem gateSubsystem;
    private final HarvesterSubsystem harvesterSubsystem;
    private final DriveSubsystem drive;
    private final CameraSubsystem camera;

    /** Telemetry for robot status information */
    private final Telemetry telemetry = StealthOpMode.telemetry;

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
        shooter = new DoubleShooterSubsystem(hardwareMap, this);
        drive = new DriveSubsystem(hardwareMap, this);
        gateSubsystem = new GateSubsystem(hardwareMap);
        harvesterSubsystem = new HarvesterSubsystem(hardwareMap);

        this.driver = driver;
        this.operator = operator;

    }

    /**
     * Configures gamepad bindings for both the driver and the operator.
     */
    public void configureBindings() {
        // --- Driver Controls ---
        drive.setDriver(driver);

        // intake is analog left reverses and right intakes
        intakeSubsystem.setDefaultCommand(intakeSubsystem.runIntakeCommand(
                () -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

        // Reset the heading only
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(drive.resetHeadingCommand());

        // Reset the position to the Scoring side corner wall
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> drive.resetPosition(false));
        // Reset the position to the audience side corner wall
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> drive.resetPosition(true));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(drive.setSlowMoCommand(true))
                .whenReleased(drive.setSlowMoCommand(false));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(drive.setAutoAimCommand(true))
                .whenReleased(drive.setAutoAimCommand(false));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shooter.startShootCommand())
                .whenReleased(shooter.stopShootCommand());

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(shooter::toggleFloatPower);



        // --- Operator Controls ---
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(shooter.adjustBoostRpmCommand(50));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(shooter.adjustBoostRpmCommand(-50));
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shooter.startShootCommand())
                .whenReleased(shooter.stopShootCommand());

        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(harvesterSubsystem::toggle);


    }


    @Override
    public void periodic(){
        updateCameraPipeline();
//        telemetry.addData("Alliance: ", Alliance.get());
//        telemetry.addData("Slow Motion", driveCmd.isSlowMo());
//        telemetry.addData("Robot Centric", driveCmd.isRobotCentric());
//        telemetry.addData("Hold Mode", driveCmd.isHoldMode());

    }
    public GateSubsystem getGateSubsystem(){
        return gateSubsystem;
    }
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
    public DoubleShooterSubsystem getShooter() {
        return shooter;
    }

    /** @return The intake subsystem */
    public IntakeSubsystem getIntake() {
        return intakeSubsystem;
    }

    /**
     * Updates the camera pipeline based on the robot's current pose.
     * Assumes the drive subsystem's follower is initialized.
     */
    public void updateCameraPipeline() {

        Pose cuPose = drive.getFollower().pose();
        camera.updateRobotOrientation(cuPose.heading());

        boolean isNear = cuPose.y() < FieldConstants.FIELD_Y_MAX / 2.0;
        Pipeline pipeline = Alliance.isBlue()
                ? (isNear ? Pipeline.BLUE_NEAR : Pipeline.BLUE_FAR)
                : (isNear ? Pipeline.RED_NEAR : Pipeline.RED_FAR);

        camera.setPipeline(pipeline);
    }
}