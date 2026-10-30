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

    private static final double SHOOT_POWER = .9;
    private static final String INTAKE_MOTOR_LEFT_NAME = "leftIntakeMotor";
    private static final String INTAKE_MOTOR_RIGHT_NAME = "rightIntakeMotor";

    private final DcMotorEx intakeLeft;
    private final DcMotorEx intakeRight;

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
        intakeLeft = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_LEFT_NAME);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeRight = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_RIGHT_NAME);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setManualPower(double p) {
        if (Math.abs(p) > POWER_THRESHOLD) {
            intakeLeft.setPower(p);
            intakeRight.setPower(p);
            currentMode = IntakeMode.MANUAL;
        } else if (!currentMode.isEqual(IntakeMode.AUTO)){
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
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
            intakeLeft.setPower(SHOOT_POWER);
            intakeRight.setPower(SHOOT_POWER);
        } else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
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
}