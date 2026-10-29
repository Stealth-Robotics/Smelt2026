package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class GateSubsystem extends StealthSubsystem {
    public final Servo servo;
    public static final double GATE_CLOSED = 0.399;
    public static final double GATE_OPEN = 0.355;
    public GateSubsystem(HardwareMap map){
        servo = map.get(Servo.class, "gateServo");
    }

    public void openGate(){
        setPosition(GATE_OPEN);
    }

    public void closeGate(){
        setPosition(GATE_CLOSED);
    }

    public Servo getServo() {
        return servo;
    }
    public void setGate(boolean open){
        if (open){
            openGate();
        }
        else {
            closeGate();
        }
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }
}
