package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import javax.annotation.Nullable;

public class RobotSystem extends StealthSubsystem {
    private GamepadEx driver;
    private GamepadEx operator;
    private final IntakeSubsystem intakeSubsystem;
    //private final ShooterSmallSubsystem shooterSmallSubsystem;
    private final SingleShooterSubsystem shooterSmallSubsystem;
    private final DriveSubsystem drive;

    private final Telemetry telemetry = StealthOpMode.telemetry;

    double rpm = 3600;

    @Nullable
    private DriveCommand driveCmd;

    public RobotSystem(HardwareMap hardwareMap) {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        //shooterSmallSubsystem = new ShooterSmallSubsystem(hardwareMap);
        shooterSmallSubsystem = new SingleShooterSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);

    }
    public RobotSystem(HardwareMap hardwareMap, GamepadEx driver, GamepadEx operator) {
        this(hardwareMap);
        this.driver = driver;
        this.operator = operator;
        configureBindings();
    }

    private void configureBindings(){
        SingleShooterSubsystem shooter = this.shooterSmallSubsystem;
        intakeSubsystem.setDefaultCommand(intakeDefaultCommand(
                ()-> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

        driveCmd = new DriveCommand(drive,
                () -> driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX());

        drive.register();
        driveCmd.addRequirements(drive);
        drive.setDefaultCommand(driveCmd);

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(()-> drive.resetOdometry(false));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(driveCmd::toggleSlowMo);
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(driveCmd::toggleRobotCentric);
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(()-> Objects.requireNonNull(driveCmd).setSlowMo(true))
                .whenReleased(()-> Objects.requireNonNull(driveCmd).setSlowMo(false));
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(driveCmd::toggleHoldMode);

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(()-> {
            shooter.setRpm(rpm);
        }).whenReleased(()-> {
            shooter.setRpm(0);
        });

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()->{
            rpm += 50;
            if (shooter.isActive()){
                shooter.setRpm(rpm);
            }
        });

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()->{
            rpm -= 50;
            if (shooter.isActive()){
                shooter.setRpm(rpm);
           }
        });

    }

    public SingleShooterSubsystem getShooterSmall() {
        return shooterSmallSubsystem;
    }

    private Command intakeDefaultCommand(DoubleSupplier intakePower) {
        return new RunCommand(() -> intakeSubsystem.setPower(intakePower.getAsDouble()),intakeSubsystem);
    }

    public DriveSubsystem getDrive() {
        return drive;
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
