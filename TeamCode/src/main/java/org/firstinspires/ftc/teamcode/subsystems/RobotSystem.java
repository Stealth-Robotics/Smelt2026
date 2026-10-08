package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

public class RobotSystem extends StealthSubsystem {

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSmallSubsystem shooterSmallSubsystem;
    private final DriveSubsystem drive;

    public RobotSystem(HardwareMap hardwareMap, GamepadEx driver, GamepadEx operator) {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSmallSubsystem = new ShooterSmallSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);

        intakeSubsystem.setDefaultCommand(intakeDefaultCommand(
                ()-> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
    }

    public ShooterSmallSubsystem getShooterSmall() {
        return shooterSmallSubsystem;
    }

    private Command intakeDefaultCommand(DoubleSupplier intakePower) {
        return new RunCommand(() -> intakeSubsystem.setPower(intakePower.getAsDouble()),intakeSubsystem);
    }
}
