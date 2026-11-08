package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.stealthrobotics.library.StealthSubsystem;

public class GateSubsystem extends StealthSubsystem {
    public final Servo servo;
    public static final double GATE_CLOSED = 0.21;
    public static final double GATE_OPEN = 0.07;
    private final ElapsedTime transitionTime = new ElapsedTime();
    private static  final double GATE_TRANSITION_TIME = 400;

    private enum GateState {
        CLOSED,
        OPENING,
        OPEN,
        CLOSING
    }

    private GateState gateState = GateState.CLOSED;


    public GateSubsystem(HardwareMap map){
        servo = map.get(Servo.class, "gateServo");
        setPosition(GATE_CLOSED);
    }

    public void open(){
        if (gateState != GateState.OPEN && gateState != GateState.OPENING) {
            transitionTime.reset();
            gateState = GateState.OPENING;
            setPosition(GATE_OPEN);
        }
    }

    public void close(){
        if (gateState != GateState.CLOSED && gateState != GateState.CLOSING) {
            transitionTime.reset();
            gateState = GateState.CLOSING;
            setPosition(GATE_CLOSED);
        }
    }

    public Servo getServo() {
        return servo;
    }

    public boolean isOpen() {
        return gateState == GateState.OPEN;
    }

    public boolean isClosed() {
        return gateState == GateState.CLOSED;
    }


    public void setPosition(double position){
            servo.setPosition(position);
    }

    @Override
    public void periodic() {

        switch (gateState) {
            case OPENING:
                if (transitionTime.milliseconds() > GATE_TRANSITION_TIME) {
                    gateState = GateState.OPEN;
                }
                break;
            case CLOSING:
                if (transitionTime.milliseconds() > GATE_TRANSITION_TIME) {
                    gateState = GateState.CLOSED;
                }
        }
    }
}
