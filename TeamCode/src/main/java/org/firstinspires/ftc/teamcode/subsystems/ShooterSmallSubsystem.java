package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

public class ShooterSmallSubsystem extends StealthSubsystem {
    private static final String MOTOR_NAME_1 = "shooterSmall";

    private static final double MOTOR_TICKS_REV = 28;

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private final DcMotorEx shooterSmallMotor;

    public ShooterSmallSubsystem(HardwareMap hardwareMap) {
        shooterSmallMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_1);
    }

    public void setPower(double power) {
        shooterSmallMotor.setPower(power);
    }

    public double getRpm() {
        return shooterSmallMotor.getVelocity() / MOTOR_TICKS_REV * 60;
    }

    @Override
    public void periodic() {
        telemetryM.addData("Small Shooter RPM", getRpm());
    }
}
