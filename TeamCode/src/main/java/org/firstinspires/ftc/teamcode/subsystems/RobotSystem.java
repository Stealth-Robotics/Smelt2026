package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

public class RobotSystem extends StealthSubsystem {
    private GamepadEx driver;
    private GamepadEx operator;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSmallSubsystem shooterSmallSubsystem;
    private final DriveSubsystem drive;

    public RobotSystem(HardwareMap hardwareMap) {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSmallSubsystem = new ShooterSmallSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);

    }
    public RobotSystem(HardwareMap hardwareMap, GamepadEx driver, GamepadEx operator) {
        this(hardwareMap);
        this.driver = driver;
        this.operator = operator;
        configureBindings();

    }

    private void configureBindings(){
        intakeSubsystem.setDefaultCommand(intakeDefaultCommand(
                ()-> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

        DriveCommand driveCmd = new DriveCommand(drive,
                () -> -driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> driver.getRightX());
        drive.register();
        driveCmd.addRequirements(drive);
        drive.setDefaultCommand(driveCmd);

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(drive::resetOdometry);
        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(driveCmd::toggleSlomo);


    }

    public ShooterSmallSubsystem getShooterSmall() {
        return shooterSmallSubsystem;
    }

    private Command intakeDefaultCommand(DoubleSupplier intakePower) {
        return new RunCommand(() -> intakeSubsystem.setPower(intakePower.getAsDouble()),intakeSubsystem);
    }
}
