package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

import org.stealthrobotics.library.opmodes.StealthOpMode;

@TeleOp(name = "TestShooter")
public class TestShooter extends StealthOpMode {
    private GamepadEx driver;
    private GamepadEx operator;
    private RobotSystem robotSystem;
    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
         robotSystem = new RobotSystem(
            hardwareMap,
                 driver,
                 operator);

         driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(robotSystem.getShooterSmall().setPowerCmd(0));
         driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(robotSystem.getShooterSmall().setPowerCmd(1));

    }
}
