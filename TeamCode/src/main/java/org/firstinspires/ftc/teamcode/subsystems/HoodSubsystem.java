package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class HoodSubsystem extends StealthSubsystem {
    private final Servo hoodServo;
    private static final double MIN_POSITION = 0;
    private static final double MAX_POSITION = 0.63;

    private InterpLUT rangeToHoodAngle;
    private static final double MIN_DISTANCE = 30;
    private static final double MAX_DISTANCE = 70;
    public double angleOverride = 0;
    private static final double DEFAULT_POSITION = 0.5;

    public HoodSubsystem(HardwareMap map) {
        hoodServo = map.get(Servo.class, "hoodServo");
        hoodServo.setPosition(DEFAULT_POSITION);
        buildRangeTable();
    }

    private void buildRangeTable() {
        // TODO: make these angles for easier readability
        rangeToHoodAngle = new InterpLUT();
        rangeToHoodAngle.add(MIN_DISTANCE - 0.1, 0.1);
        rangeToHoodAngle.add(40, 0.13);
        rangeToHoodAngle.add(50, .25);
        rangeToHoodAngle.add(55, .33);
        rangeToHoodAngle.add(60, .4);
        rangeToHoodAngle.add(MAX_DISTANCE + 0.1, MAX_POSITION
        );
        rangeToHoodAngle.createLUT();
    }

    public void setPosition(double pos) {
        pos += angleOverride;
        pos = Math.clamp(pos, MIN_POSITION, MAX_POSITION);
        hoodServo.setPosition(pos);
    }

    public void setHoodAngle(double distance) {
        double angle = getHoodAngleFromRange(distance);
        setPosition(angle);
    }

    // Gets the raw servo Caution should be used with this.
    public Servo getServo() {
        return hoodServo;
    }

    private double getHoodAngleFromRange(double distance) {
        if (distance == 0) {
            return 0;
        }

        double dist = Math.clamp(distance, MIN_DISTANCE, MAX_DISTANCE);
        return rangeToHoodAngle.get(dist);
    }

    // This will just return whatever hangle was requested by setPosition
    public double getHoodAngle() {
        return hoodServo.getPosition();
    }

    public void setAngleOverride(double angle) {
        angleOverride = angle;
    }

    public void incrementAngleOverride(double amount) {
        angleOverride += amount;
    }

    public double getAngleOverride() {
        return angleOverride;
    }

}
