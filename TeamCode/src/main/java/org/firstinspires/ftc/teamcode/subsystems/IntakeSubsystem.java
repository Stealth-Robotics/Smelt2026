package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx IntakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotorEx.class,"testMotor");
    }

    public void setPower(double p) {
        IntakeMotor.setPower(p);
    }

    @Override
    public void periodic() {

    }
}
