package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class SlabSubsystem extends StealthSubsystem {
    private final Servo slabServo;
    private static final double SLAB_UP = 0.35;
    private static final double SLAB_BLOCK = 0.03;
    private double lastPosition = SLAB_UP;

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
        return lastPosition;
    }

    public void setBlock(){
        lastPosition = SLAB_BLOCK;
        this.slabServo.setPosition(SLAB_BLOCK);
    }

    public void setUp(){
        lastPosition = SLAB_UP;
        this.slabServo.setPosition(SLAB_UP);
    }

    public void toggle(){
        if (lastPosition == SLAB_UP){
            slabServo.setPosition(SLAB_BLOCK);
            lastPosition = SLAB_BLOCK;
        }
        else{
            slabServo.setPosition(SLAB_UP);
            lastPosition = SLAB_UP;
        }
    }

    public Command toggleCmd(){
        return runOnce(this::toggle);
    }
}
