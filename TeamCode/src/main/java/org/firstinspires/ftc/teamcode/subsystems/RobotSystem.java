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
    private final DmShooterSubsystem shooter;
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
        shooter = new DmShooterSubsystem(hardwareMap, this);
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
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(drive.resetHeadingCmd());

        // Reset the position to the Scoring side corner wall
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> drive.resetPosition(false));
        // Reset the position to the audience side corner wall
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> drive.resetPosition(true));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(drive.setSlowMoCmd(true))
                .whenReleased(drive.setSlowMoCmd(false));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(drive.setAutoAimCmd(true))
                .whenReleased(drive.setAutoAimCmd(false));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shooter.startShootCmd())
                .whenReleased(shooter.stopShootCmd());

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(shooter::toggleFloatPower);



        // --- Operator Controls ---
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(shooter.adjustBoostRpmCmd(50));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(shooter.adjustBoostRpmCmd(-50));
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shooter.startShootCmd())
                .whenReleased(shooter.stopShootCmd());

        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(harvesterSubsystem::toggle);


    }


    @Override
    public void periodic(){
        updateCameraPipeline();

//        telemetry.addData("Alliance: ", Alliance.get());
//        telemetry.addData("Slow Motion", drive.isSlowMo());
//        telemetry.addData("Robot Centric", drive.isRobotCentric());
//        telemetry.addData("Hold Mode", driveCmd.isHoldMode());
        Pose pose = drive.getFollower().pose();
        telemetry.addData("Current Pose", "Tx: %.2f, Ty: %.2f, Z: %.2f", pose.x(), pose.y(),  Math.toDegrees(pose.heading()));

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
    public DmShooterSubsystem getShooter() {
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