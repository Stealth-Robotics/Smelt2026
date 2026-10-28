package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class HarvesterSubsystem extends StealthSubsystem {
    //placeholder, still working on the other system ._.
    public final Servo servo;
    public HarvesterSubsystem(HardwareMap map){
        servo = map.get(Servo.class, "harvesterServo");
    }

    public Servo getServo() {
        return servo;
    }

    public void setPosition(double position) { servo.setPosition(position); }
}
