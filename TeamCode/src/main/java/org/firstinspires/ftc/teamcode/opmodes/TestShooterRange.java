package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleShooterSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@TeleOp(name = "Test Shooter Range Map", group = "Tests")
public class TestShooterRange extends StealthOpMode {
    private GamepadEx driver;
    private GamepadEx operator;
    private RobotSystem robot;
    private double hoodAngleAdd = 0;
    private double rpmboost = 0;
    private static final double[] hoodSteps = {0.1, 0.01, 0.001};
    private static final double[] rpmSteps = {500, 100, 50};
    private int stepIndex = 1;

    SingleShooterSubsystem shooter;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new RobotSystem(
                hardwareMap,
                driver,
                operator);
        robot.getDrive().setDriver(driver);
        shooter = robot.getShooter();
        configBindings();

        schedule(this.printInfo());
    }

    private void configBindings() {
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shooter.startShootCommand())
                .whenReleased(shooter.stopShootCommand());

        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            stepIndex = (stepIndex + 1) % hoodSteps.length;
        });
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            stepIndex = (stepIndex - 1) % hoodSteps.length;
        });
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
            hoodAngleAdd += hoodSteps[stepIndex];
            shooter.getHood().setAngleOverride(hoodAngleAdd);
        });

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            hoodAngleAdd -= hoodSteps[stepIndex];
            shooter.getHood().setAngleOverride(hoodAngleAdd);
        });

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            rpmboost += rpmSteps[stepIndex];
            shooter.setBoostRpm(rpmboost);
        });

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            rpmboost -= rpmSteps[stepIndex];
            shooter.setBoostRpm(rpmboost);
        });

    }

    public Command printInfo() {
        return new RunCommand(() -> {
            telemetry.addData("Range ", hoodAngleAdd);
            telemetry.addData("RPM Boost", rpmboost);
            telemetry.addData("Hood Angle", shooter.getHood().getHoodAngle());
            telemetry.addData("Shooter Target RPM", shooter.getTargetRpm());
            telemetry.addData("Shooter RPM", shooter.getCurrentRpm());
            telemetry.addData("Step Index", stepIndex);
            telemetry.addData("Step Value", "%.5f", hoodSteps[stepIndex]);
            telemetry.addData("Step Value", "%.5f", rpmSteps[stepIndex]);
        });
    }
}