package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

import org.stealthrobotics.library.opmodes.StealthOpMode;

@TeleOp(name = "Teleop")
public class Teleop extends StealthOpMode {
    private GamepadEx driver;
    private RobotSystem robotSystem;
    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
         robotSystem = new RobotSystem(
            hardwareMap,
            () -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
         );
//
//        ShooterSmallSubsystem shooterSmall = new ShooterSmallSubsystem(hardwareMap);
//        ShooterSmallDefaultCommand shooterSmallCmd = new ShooterSmallDefaultCommand(
//                shooterSmall,
//                () -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
//                () -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
//        shooterSmall.setDefaultCommand(shooterSmallCmd);

    }
}
