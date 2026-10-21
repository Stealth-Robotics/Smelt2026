package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import javax.annotation.Nullable;

public class RobotSystem extends StealthSubsystem {
    private GamepadEx driver;
    private GamepadEx operator;
    private IntakeSubsystem intakeSubsystem;
    //private final ShooterSmallSubsystem shooterSmallSubsystem;
    private SingleShooterSubsystem shooter;
    private DriveSubsystem drive;
    private CameraSubsystem camera;
    private ShootCommand shootCommand;

    private final Telemetry telemetry = StealthOpMode.telemetry;

    private final DriveCommand driveCmd;

    public RobotSystem(HardwareMap hardwareMap) {
        initCommon(hardwareMap);
        driveCmd = new DriveCommand(this);

        drive.register();
        driveCmd.addRequirements(drive);
        drive.setDefaultCommand(driveCmd);
    }

    public RobotSystem(HardwareMap hardwareMap, GamepadEx driver, GamepadEx operator) {
        initCommon(hardwareMap);
        this.driver = driver;
        this.operator = operator;
        driveCmd = new DriveCommand(this,
                () -> driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX());

        drive.register();
        driveCmd.addRequirements(drive);
        drive.setDefaultCommand(driveCmd);
    }

    private void initCommon(HardwareMap hardwareMap) {
        camera = new CameraSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooter = new SingleShooterSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);

        shootCommand = new ShootCommand(this);
        shooter.setDefaultCommand(shootCommand);
    }

    public void configureBindings(){
        intakeSubsystem.setDefaultCommand(intakeSubsystem.runIntakeCommand(
                () -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(drive.resetOdometryCommand(false));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(driveCmd.toggleSlowMoCommand());
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(driveCmd.toggleRobotCentricCommand());
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(driveCmd.setSlowMoCommand(true))
                .whenReleased(driveCmd.setSlowMoCommand(false));
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(driveCmd.toggleHoldModeCommand());

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(driveCmd.setAutoAimCommand(true))
                .whenReleased(driveCmd.setAutoAimCommand(false));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shootCommand.startShootCommand())
                .whenReleased(shootCommand.stopShootCommand());

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(shooter.adjustBoostRpmCommand(50));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(shooter.adjustBoostRpmCommand(-50));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(shooter.resetBoostRpmCommand());

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(shooter.adjustBoostRpmCommand(50));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(shooter.adjustBoostRpmCommand(-50));
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shootCommand.startShootCommand())
                .whenReleased(shootCommand.stopShootCommand());

        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(drive.resetOdometryCommand(true));
    }

    public SingleShooterSubsystem getShooterSmall() {
        return shooter;
    }

    public ShootCommand getShootCommand() {
        return shootCommand;
    }

    public DriveSubsystem getDriveSubsystem() {
        return drive;
    }

    public CameraSubsystem getCamera() {
        return camera;
    }

    public SingleShooterSubsystem getShooter() {
        return shooter;
    }

    public DriveCommand getDriveCommand() {
        return driveCmd;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    @Override
    public void periodic(){
        if (drive != null && camera != null) {
            updateCameraPipeline();
        }
        
        if (driveCmd != null){
//            telemetry.addData("Alliance: ", Alliance.get());
//            telemetry.addData("Slow Motion", driveCmd.isSlowMo());
//            telemetry.addData("Robot Centric", driveCmd.isRobotCentric());
//            telemetry.addData("Hold Mode", driveCmd.isHoldMode());
        }
    }

    public void updateCameraPipeline() {
        Pose pose = drive.getFollower().pose();
        camera.updateRobotOrientation(pose.heading());

        boolean isNear = pose.y() < FieldConstants.FIELD_Y_MAX / 2.0;
        org.firstinspires.ftc.teamcode.tools.Pipeline pipeline = Alliance.isBlue()
                ? (isNear ? org.firstinspires.ftc.teamcode.tools.Pipeline.BLUE_NEAR : org.firstinspires.ftc.teamcode.tools.Pipeline.BLUE_FAR)
                : (isNear ? org.firstinspires.ftc.teamcode.tools.Pipeline.RED_NEAR : org.firstinspires.ftc.teamcode.tools.Pipeline.RED_FAR);

        camera.setPipeline(pipeline);
    }
}
