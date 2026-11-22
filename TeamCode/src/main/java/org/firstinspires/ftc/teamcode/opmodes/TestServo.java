package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HarvesterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.SlabSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@TeleOp(name = "TestServo Slab", group = "Tests")
public class TestServo extends StealthOpMode {
    private GamepadEx driver;
    private GamepadEx operator;
    private RobotSystem robotSystem;

    double position = .3678;

    private Servo servo;

    private static final double[] stepSizes = {0.1, 0.01, 0.001};
    private int stepIndex = 1;

    @Override
    public void initialize() {
        stepIndex = 1;
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robotSystem = new RobotSystem(
                hardwareMap,
                driver,
                operator);
        // Change Servo here to test different servos
        //servo = new HarvesterSubsystem(hardwareMap).getServo();
        //servo = new HoodSubsystem(hardwareMap).getServo();
        servo = new SlabSubsystem(hardwareMap).getServo();
        servo.setPosition(position);
        configBindings();
        schedule(this.printInfo());
    }

    private void configBindings() {
        robotSystem.configDriverBindings();

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()-> {
            position += stepSizes[stepIndex];
            servo.setPosition(position);
        });

        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        });

        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            stepIndex = (stepIndex - 1 + stepSizes.length) % stepSizes.length;
        });

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()->{
            position -= stepSizes[stepIndex];
            // servos can't go bellow 0
            if (position < 0) {
                position = 0;
            }
            servo.setPosition(position);
        });

        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            servo.setPosition(0);
        });

        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            servo.setPosition(0.97);
        });

    }

    public Command printInfo() {
        return new RunCommand(() ->{
            telemetry.addData("Step Index", stepIndex);
            telemetry.addData("Step Value", "%.3f", stepSizes[stepIndex]);
            telemetry.addData("Position", position);
//            telemetry.addData( "Angle", robotSystem.getShooter().getHood().getAngle());
//            telemetry.addData("Hood Pose", robotSystem.getShooter().getHood().getPosition());
            telemetry.addData("Servo Position", servo.getPosition());
        });
    }
}

