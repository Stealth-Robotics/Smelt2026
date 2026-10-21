package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
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

    private DriveCommand driveCmd;

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
        //shooterSmallSubsystem = new ShooterSmallSubsystem(hardwareMap);
        shooter = new SingleShooterSubsystem(hardwareMap);
        shootCommand = new ShootCommand(this);
        shootCommand.addRequirements(shooter);
        shooter.setDefaultCommand(shootCommand);
        drive = new DriveSubsystem(hardwareMap);
    }

    public void configureBindings(){
        intakeSubsystem.setDefaultCommand(intakeDefaultCommand(
                ()-> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));



        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(()-> drive.resetOdometry(false));
        // TODO:a number of these commands should be moved to operator
        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(driveCmd::toggleSlowMo);
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(driveCmd::toggleRobotCentric);
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(()-> Objects.requireNonNull(driveCmd).setSlowMo(true))
                .whenReleased(()-> Objects.requireNonNull(driveCmd).setSlowMo(false));
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(driveCmd::toggleHoldMode);

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(()-> Objects.requireNonNull(driveCmd).setAutoAim(true))
                .whenReleased(()-> Objects.requireNonNull(driveCmd).setAutoAim(false));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> shootCommand.setStartShoot(true))
                .whenReleased(() -> shootCommand.setStartShoot(false));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() ->
            shooter.setBoostRpm(shooter.getBoostRpm() + 50));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() ->
                shooter.setBoostRpm(shooter.getBoostRpm() - 50));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(shooter::resetBoostRpm);

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() ->
                shooter.setBoostRpm(shooter.getBoostRpm() + 50));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() ->
                shooter.setBoostRpm(shooter.getBoostRpm() - 50));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> shootCommand.setStartShoot(true))
                .whenReleased(() -> shootCommand.setStartShoot(false));

        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> drive.resetOdometry(true));
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

    private Command intakeDefaultCommand(DoubleSupplier intakePower) {
        return new RunCommand(() -> intakeSubsystem.setPower(intakePower.getAsDouble()),intakeSubsystem);
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    @Override
    public void periodic(){
        if (driveCmd != null){
//            telemetry.addData("Alliance: ", Alliance.get());
//            telemetry.addData("Slow Motion", driveCmd.isSlowMo());
//            telemetry.addData("Robot Centric", driveCmd.isRobotCentric());
//            telemetry.addData("Hold Mode", driveCmd.isHoldMode());
        }
    }
}
