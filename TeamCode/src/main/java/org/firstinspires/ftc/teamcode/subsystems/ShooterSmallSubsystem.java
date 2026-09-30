package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class ShooterSmallSubsystem extends StealthSubsystem {
    private static final String MOTOR_NAME_1 = "shooterSmall";
    private final Telemetry telemetry = StealthOpMode.telemetry;

    private static final double MOTOR_TICKS_REV = 28;


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
        telemetry.addData("Small Shooter RPM", getRpm());
    }
}
