package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@TeleOp(name = "StealthTele")
public class StealthOp extends StealthOpMode {
    private GamepadEx driver;
    private GamepadEx operator;
    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        RobotSystem robot = new RobotSystem(hardwareMap, driver, operator);

    }
}
