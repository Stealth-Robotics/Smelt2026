package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DmShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.tools.Targets;
import org.stealthrobotics.library.opmodes.StealthOpMode;


public class TestShooterRange extends StealthOpMode {
    private GamepadEx driver;
    private GamepadEx operator;
    private RobotSystem robot;
    private double hoodAngleAdd = 0;
    private double rpmboost = 0;
    private static final double[] hoodSteps = {0.1, 0.01, 0.001};
    private static final double[] rpmSteps = {100, 50, 25};
    private int stepIndex = 1;

    DmShooterSubsystem shooter;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new RobotSystem(
                hardwareMap,
                driver,
                operator);

        shooter = robot.getShooter();
        configBindings();

        schedule(this.printInfo());
    }

    private void configBindings() {

        robot.configDriverBindings();

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shooter.startShootCmd())
                .whenReleased(shooter.stopShootCmd());

        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            stepIndex = (stepIndex + 1) % hoodSteps.length;
        });

        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            stepIndex = (stepIndex - 1 + hoodSteps.length) % hoodSteps.length;
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
            telemetry.addData("Camera Distance", CameraSubsystem.calcGoalDistanceByTa(robot.getCamera().getLastResult()));
            telemetry.addData("Pose Distance", Targets.getDistanceToTarget(robot.getDrive().getPose()));
            telemetry.addData("Shooter Target RPM", shooter.getTargetRpm());
            telemetry.addData("Shooter RPM", shooter.getCurrentRpm());
            telemetry.addData("Step Value", "%.5f", rpmSteps[stepIndex]);
            telemetry.addData("RPM Override", rpmboost);
            telemetry.addLine();
            telemetry.addData("Hood Angle", shooter.getHood().getAngle());
            telemetry.addData("Hood Position", shooter.getHood().getPosition());
            telemetry.addData("Step Value", "%.5f", hoodSteps[stepIndex]);
            telemetry.addData("Hood Angle Override ", hoodAngleAdd);
        });
    }

    @TeleOp(name = "Test Shooter Range Map RED", group = "Tests")
    public static class TestShootRed extends TestShooterRange{
    }

    @TeleOp(name = "Test Shooter Range Map BLUE", group = "Tests")
    public static class TestShootBlue extends TestShooterRange{
    }
}