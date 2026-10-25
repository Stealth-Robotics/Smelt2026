package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends StealthSubsystem {
    private static final double POWER_THRESHOLD = 0.03;

    private static final double MAX_INTAKE_POWER = 0.7;
    private static final String INTAKE_MOTOR_NAME = "intakeMotor";

    private final DcMotorEx intake;

    public enum IntakeMode {
        OFF,
        MANUAL,
        AUTO;

        public boolean isEqual(IntakeMode mode) {
            return this == mode;
        }
    }
    private IntakeMode currentMode = IntakeMode.OFF;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setManualPower(double p) {
        if (Math.abs(p) > POWER_THRESHOLD) {
            intake.setPower(p);
            currentMode = IntakeMode.MANUAL;
        } else {
            intake.setPower(0);
            currentMode = IntakeMode.OFF;
        }
    }

    @Override
    public void periodic() {
//        telemetry.addData("Intake Power:", intake.getPower());
//        telemetry.addData("Intake Mode:", currentMode);
    }

    public boolean getAutoIntake() {
        return currentMode == IntakeMode.AUTO;
    }

    public void setAutoIntake(boolean run) {
        if (currentMode.isEqual(IntakeMode.MANUAL)) {
            return;
        }

        currentMode = run ? IntakeMode.AUTO : IntakeMode.OFF;

        if (run) {
            intake.setPower(MAX_INTAKE_POWER);
        } else {
            intake.setPower(0);
        }
    }

    public Command setAutoIntakeCommand(boolean run) {
        return runOnce(() -> setAutoIntake(run));
    }

    public Command startAutoIntakeCommand() {
        return setAutoIntakeCommand(true);
    }

    public Command stopAutoIntakeCommand() {
        return setAutoIntakeCommand(false);
    }

    public Command runIntakeCommand(DoubleSupplier powerSupplier) {
        return run(() -> setManualPower(powerSupplier.getAsDouble()));
    }
}