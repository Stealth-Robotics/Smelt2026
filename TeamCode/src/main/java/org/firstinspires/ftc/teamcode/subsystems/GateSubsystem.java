package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class GateSubsystem extends StealthSubsystem {
    public final Servo servo;
    public GateSubsystem(HardwareMap map){
        servo = map.get(Servo.class, "gateServo");
    }

    public Servo getServo() {
        return servo;
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }
}
