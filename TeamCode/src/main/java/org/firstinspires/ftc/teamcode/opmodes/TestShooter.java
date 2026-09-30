package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ShooterSmallDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSmallSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@TeleOp(name = "TestShooter")
public class TestShooter extends StealthOpMode {

    public GamepadEx driver;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);

        ShooterSmallSubsystem shooterSmall = new ShooterSmallSubsystem(hardwareMap);
        ShooterSmallDefaultCommand shooterSmallCmd = new ShooterSmallDefaultCommand(
                shooterSmall,
                () -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        shooterSmall.setDefaultCommand(shooterSmallCmd);
    }

}
