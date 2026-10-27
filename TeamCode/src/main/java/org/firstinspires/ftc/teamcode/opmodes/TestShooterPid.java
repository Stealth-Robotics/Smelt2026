package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

import org.firstinspires.ftc.teamcode.subsystems.SingleShooterSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@TeleOp(name = "Test Shooter PID", group = "Tests")
public class TestShooterPid extends StealthOpMode {
    private GamepadEx driver;
    private GamepadEx operator;
    private RobotSystem robotSystem;

    private double kf = .0;
    private double kp = .0;

    private  double rpm = 3400;

    private static final double coast = 500;

    private static final double[] stepSizes = {0.01, 0.001, 0.0001, 0.00001, 0.000001};
    private int stepIndex = 1;

    private boolean shootToggle = false;

    @Override
    public void initialize() {
        stepIndex = 1;
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
         robotSystem = new RobotSystem(
            hardwareMap,
                 driver,
                 operator);
        robotSystem.getDrive().setDriver(driver);
        configBindings();
        robotSystem.getShooter().setController(kp, 0,0, kf);
        schedule(this.printInfo());
    }

    private void configBindings() {

        SingleShooterSubsystem shooter = robotSystem.getShooter();
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(()->{
            shootToggle = !shootToggle;
            if (shootToggle) {
                shooter.setStartShoot(true);
            }else {
                shooter.stop();
                shooter.setFloatPower(0);
            }
        });

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(()->{
            shooter.setRpm(rpm);
            shootToggle = true;
        }).whenReleased(()-> {
            shooter.stop();
            shooter.setFloatPower(0);
            shootToggle = false;
        });

        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
           stepIndex = (stepIndex + 1) % stepSizes.length;
        });

        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            stepIndex = (stepIndex - 1 + stepSizes.length) % stepSizes.length;
        });

        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            rpm += 50;
            shooter.setRpm(rpm);
        });

        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            rpm -= 50;
            shooter.setRpm(rpm);
        });

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(()-> {
           kf += stepSizes[stepIndex];
            shooter.setController(kp, 0, 0, kf);
        });

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(()->{
            kf -= stepSizes[stepIndex];
            shooter.setController(kp, 0, 0, kf);
        });

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()-> {
            kp += stepSizes[stepIndex];
            shooter.setController(kp, 0, 0, kf);
        });

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()->{
            kp -= stepSizes[stepIndex];
            shooter.setController(kp, 0, 0, kf);
        });

    }

    public Command printInfo() {
        return new RunCommand(() ->{
           telemetry.addData("Step Index", stepIndex);
           telemetry.addData("Step Value", "%.6f", stepSizes[stepIndex]);
           telemetry.addData("KP","%.6f", kp);
           telemetry.addData("KF","%.6f", kf);

        });
    }
}
