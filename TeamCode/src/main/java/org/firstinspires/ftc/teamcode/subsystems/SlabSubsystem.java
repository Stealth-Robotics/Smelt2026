package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class SlabSubsystem extends StealthSubsystem {
    private final Servo slabServo;
    private static final double SLAB_UP = 0.34;
    private static final double SLAB_BLOCK = 0.07;


    public SlabSubsystem(HardwareMap map) {
        slabServo = map.get(Servo.class, "SlabServo");
        slabServo.setPosition(SLAB_UP);
    }

    public Servo getServo() {
        return slabServo;
    }

    public void setPosition(double position) {
        position = Math.clamp(SLAB_BLOCK, SLAB_UP, position);
        slabServo.setPosition(position);
    }

    public double getPosition() {
        return slabServo.getPosition();
    }

    public void setBlock(){
        setPosition(SLAB_BLOCK);
    }

    public void setUp(){
        setPosition(SLAB_UP);
    }

    public void toggle(){
        if (getPosition() == SLAB_BLOCK){
            setUp();
        }
        else{
            setBlock();
        }
    }

    public Command toggleCmd(){
        return runOnce(this::toggle);
    }
}
