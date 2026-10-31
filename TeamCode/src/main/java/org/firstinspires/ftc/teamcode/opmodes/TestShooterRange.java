package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DmShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
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
    private static final double[] distanceSteps = {10, 5, 1};
    private int stepIndex = 1;

    private double distanceOverride = 0;

    DmShooterSubsystem shooter;

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
        IntakeSubsystem intakeSubsystem = robot.getIntake();
        DriveSubsystem drive = robot.getDrive();

        intakeSubsystem.setDefaultCommand(intakeSubsystem.runIntakeCommand(
                () -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(drive.resetHeadingCmd());
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()-> drive.resetPosition(false));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()-> drive.resetPosition(true));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(drive.setSlowMoCmd(true))
                .whenReleased(drive.setSlowMoCmd(false));
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(drive.toggleHoldModeCmd());

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(drive.setAutoAimCmd(true))
                .whenReleased(drive.setAutoAimCmd(false));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shooter.startShootCmd(0))
                .whenReleased(shooter.stopShootCmd());

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

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(() -> {
            distanceOverride = 50;
            shooter.setDistance(distanceOverride);
        });

        operator.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(() -> {
            distanceOverride = 0;
            shooter.setDistance(distanceOverride);
        });

        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            distanceOverride += distanceSteps[stepIndex];
            shooter.setDistance(distanceOverride);
        });

        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            distanceOverride -= distanceSteps[stepIndex];
            shooter.setDistance(distanceOverride);
        });


    }

    public Command printInfo() {
        return new RunCommand(() -> {
            telemetry.addData("Shooter Target RPM", shooter.getTargetRpm());
            telemetry.addData("Shooter RPM", shooter.getCurrentRpm());
            telemetry.addData("Step Value", "%.5f", rpmSteps[stepIndex]);
            telemetry.addData("RPM Override", rpmboost);
            telemetry.addLine();
            telemetry.addData("Hood Angle", shooter.getHood().getHoodAngle());
            telemetry.addData("Step Value", "%.5f", hoodSteps[stepIndex]);
            telemetry.addData("Hood Angle Override ", hoodAngleAdd);
            telemetry.addLine();
            telemetry.addData("Distance Override", distanceOverride);
            telemetry.addData("Step Value", "%.5f", distanceSteps[stepIndex]);

        });
    }
}