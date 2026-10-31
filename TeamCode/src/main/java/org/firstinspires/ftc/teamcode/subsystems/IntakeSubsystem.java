package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends StealthSubsystem {
    private static final double POWER_THRESHOLD = 0.05;

    private static final double SHOOT_POWER = 1.0;
    private static final double UNJAM_POWER = -1.0;

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
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setManualPower(double p) {
        if (Math.abs(p) > POWER_THRESHOLD) {
            intake.setPower(p);
            currentMode = IntakeMode.MANUAL;
        } else if (!currentMode.isEqual(IntakeMode.AUTO)){
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
            intake.setPower(SHOOT_POWER);
        } else {
            intake.setPower(0);
        }
    }

    public Command setAutoIntakeCommand(boolean run) {
        return runOnce(() -> setAutoIntake(run));
    }

    public Command startAutoIntakeCommand(boolean deployPollenWedge) {
        // TODO: build this out with servo and pollen wedge
        return setAutoIntakeCommand(true);
    }

    public Command stopAutoIntakeCommand() {
        // TODO: Retract pollen wedge if out
        return setAutoIntakeCommand(false);
    }

    public Command runIntakeCommand(DoubleSupplier powerSupplier) {
        return run(() -> setManualPower(powerSupplier.getAsDouble()));
    }

    public void setAutoOuttake(boolean run) {

        currentMode = run ? IntakeMode.AUTO : IntakeMode.OFF;
        if (run) {
            intake.setPower(UNJAM_POWER);
        } else {
            intake.setPower(0);
        }
    }
    public Command setAutoOuttakeCommand(boolean run) {
        return runOnce(() -> setAutoOuttake(run));
    }
}