package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.VelocityShooterSubsystem;
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

    VelocityShooterSubsystem shooter;

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
        VelocityShooterSubsystem shooter = robot.getShooter();
        IntakeSubsystem intakeSubsystem = robot.getIntake();
        DriveSubsystem drive = robot.getDrive();

        intakeSubsystem.setDefaultCommand(intakeSubsystem.runIntakeCommand(
                () -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(drive.resetOdometryCommand(true));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(drive.setSlowMoCommand(true))
                .whenReleased(drive.setSlowMoCommand(false));
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(drive.toggleHoldModeCommand());

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(drive.setAutoAimCommand(true))
                .whenReleased(drive.setAutoAimCommand(false));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shooter.startShootCommand())
                .whenReleased(shooter.stopShootCommand());

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shooter.startShootCommand())
                .whenReleased(shooter.stopShootCommand());

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

        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            drive.resetOdometry(true);
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