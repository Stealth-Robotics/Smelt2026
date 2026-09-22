package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorSubsystem extends SubsystemBase {
    private final DcMotorEx testMotor;

    public MotorSubsystem(HardwareMap hardwareMap) {
        testMotor = hardwareMap.get(DcMotorEx.class,"testMotor");
    }

    public void setPower(double p) {
        testMotor.setPower(p);
    }

    @Override
    public void periodic() {
    }
}
