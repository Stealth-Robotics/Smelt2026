package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class HarvesterSubsystem extends StealthSubsystem {
    public final Servo servo;
    public static final double HARVESTER_UP = 0.08;
    public static final double HARVESTER_DOWN = 0.71;
    public HarvesterSubsystem(HardwareMap map){
        servo = map.get(Servo.class, "pollenServo");
        setPosition(HARVESTER_UP);
    }
    public void toggle(){
        if (servo.getPosition() == HARVESTER_UP){
            setPosition(HARVESTER_DOWN);
        }
        else {
            setPosition(HARVESTER_UP);
        }
    }

    public void Up(){
        setPosition(HARVESTER_UP);
    }
    public void Down(){
        setPosition(HARVESTER_DOWN);
    }

    public Servo getServo() {
        return servo;
    }

    public void setPosition(double position) {
        position = Math.clamp(HARVESTER_DOWN, HARVESTER_UP, position);
        servo.setPosition(position); }

    public Command harvDownCmd(){
        return runOnce(this::Down);
    }
    public Command harvUpCmd(){
        return runOnce(this::Up);
    }


}

