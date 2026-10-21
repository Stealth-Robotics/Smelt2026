package org.firstinspires.ftc.teamcode.subsystems;

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
    private final DcMotorEx intake;

    private boolean autoIntake = false;
    private boolean manualControl = false;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double p) {
        if (Math.abs(p) > .03) {
            manualControl = true;
            setAutoIntake(false);
            intake.setPower(p);
            return;
        }

        if (manualControl && Math.abs(p) < .03) {
            manualControl = false;
            intake.setPower(0);
        }

    }

    @Override
    public void periodic() {
//        telemetry.addData("Intake Power:", intake.getPower());
//        telemetry.addData("Auto Intake:", autoIntake);
//        telemetry.addData("Manual Control:", manualControl);
    }

    public boolean getAutoIntake() {
        return autoIntake;
    }

    public boolean getManualControl() {
        return manualControl;
    }

    public void setAutoIntake(boolean run) {
        if (manualControl){
            autoIntake = false;
            return;
        }

        autoIntake = run;
        if (autoIntake) {
            intake.setPower(1);
        } else if (intake.getPower() > 0.001) {
            intake.setPower(0);
        }
    }

    public Command setAutoIntakeCommand(boolean run) {
        return new InstantCommand(() -> setAutoIntake(run), this);
    }

    public Command startAutoIntakeCommand() {
        return setAutoIntakeCommand(true);
    }

    public Command stopAutoIntakeCommand() {
        return setAutoIntakeCommand(false);
    }

    public Command runIntakeCommand(DoubleSupplier powerSupplier) {
        return new RunCommand(() -> setPower(powerSupplier.getAsDouble()), this);
    }
}

