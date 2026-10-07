package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

public class RobotSystem extends StealthSubsystem {

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSmallSubsystem shooterSmallSubsystem;

    public RobotSystem(HardwareMap hardwareMap, DoubleSupplier intakeSupplier) {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSmallSubsystem = new ShooterSmallSubsystem(hardwareMap);

        intakeSubsystem.setDefaultCommand(intakeDefaultCommand(intakeSupplier));
    }

    public ShooterSmallSubsystem getShooterSmall() {
        return shooterSmallSubsystem;
    }

    private Command intakeDefaultCommand(DoubleSupplier intakePower) {
        return new RunCommand(() -> intakeSubsystem.setPower(intakePower.getAsDouble()),intakeSubsystem);
    }
}
