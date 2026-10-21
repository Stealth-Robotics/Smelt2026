package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

import org.firstinspires.ftc.teamcode.subsystems.SingleShooterSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@TeleOp(name = "TestAutoAim")
public class TestAutoAim extends StealthOpMode {
    private GamepadEx driver;
    private GamepadEx operator;
    private RobotSystem robotSystem;

    private int kIndex = 0;
    private double[] kValues = {.02, .0001, 0.15, 0.0015, 0.00};
    private static final String[] kKeys = { "Kp", "KiFar", "KiNear", "Kd", "Kf" };

    private static final double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex = 0;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
         robotSystem = new RobotSystem(
            hardwareMap,
                 driver,
                 operator);

        kIndex = 0;
        stepIndex = 0;
        configBindings();
        robotSystem.getDriveCommand().setAimController(kValues[0], kValues[1], kValues[2], kValues[3], kValues[4]);
        schedule(this.printInfo());
    }

    private void configBindings() {

        DriveCommand driveCmd = robotSystem.getDriveCommand();
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(()->{
            kIndex = (kIndex + 1) % kKeys.length;
        });

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(()->{
            kIndex = (kIndex - 1 + kKeys.length) % kKeys.length;
        });

        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
           stepIndex = (stepIndex + 1) % stepSizes.length;
        });

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            stepIndex = (stepIndex - 1 + stepSizes.length) % stepSizes.length;
        });

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            driveCmd.setAutoAim(true);
        }).whenReleased(() -> driveCmd.setAutoAim(false));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()-> {
            kValues[kIndex] += stepSizes[stepIndex];
            driveCmd.setAimController(kValues[0], kValues[1], kValues[2], kValues[3], kValues[4]);
        });

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()->{
            kValues[kIndex] -= stepSizes[stepIndex];
            driveCmd.setAimController(kValues[0], kValues[1], kValues[2], kValues[3], kValues[4]);
        });

    }

    public Command printInfo() {
        return new RunCommand(() ->{
           telemetry.addData("Step Index", stepIndex);
           telemetry.addData("Step Value", "%.5f", stepSizes[stepIndex]);
           telemetry.addData("Selected", kKeys[kIndex]);
           telemetry.addData("K Values", "Kp %.5f, KiFar %.5f, KiNear %.5f, Kd %.5f, Kf %.5f", kValues[0], kValues[1], kValues[2], kValues[3], kValues[4]);
           telemetry.addData("Current kI", robotSystem.getDriveCommand().getAimController().getI());
        });
    }
}
