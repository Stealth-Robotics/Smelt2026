package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ShooterSmallDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSmallSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@TeleOp(name = "Teleop")
public class Teleop extends StealthOpMode {
    private final GamepadEx driver = new GamepadEx(gamepad1);
    @Override
    public void initialize() {

        ShooterSmallSubsystem shooterSmall = new ShooterSmallSubsystem(hardwareMap);
        ShooterSmallDefaultCommand shooterSmallCmd = new ShooterSmallDefaultCommand(
                shooterSmall,
                () -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        shooterSmall.setDefaultCommand(shooterSmallCmd);
    }
}
