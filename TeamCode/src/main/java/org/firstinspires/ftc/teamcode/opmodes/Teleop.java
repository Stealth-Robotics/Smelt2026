package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.ShooterSmallDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSmallSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@TeleOp(name = "Teleop")
public class Teleop extends StealthOpMode {
    private final GamepadEx driver = new GamepadEx(gamepad1);
    private final RobotSystem robotSystem= new RobotSystem(
        hardwareMap,
            () -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
    );
    @Override
    public void initialize() {
//
//        ShooterSmallSubsystem shooterSmall = new ShooterSmallSubsystem(hardwareMap);
//        ShooterSmallDefaultCommand shooterSmallCmd = new ShooterSmallDefaultCommand(
//                shooterSmall,
//                () -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
//                () -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
//        shooterSmall.setDefaultCommand(shooterSmallCmd);

    }
}
