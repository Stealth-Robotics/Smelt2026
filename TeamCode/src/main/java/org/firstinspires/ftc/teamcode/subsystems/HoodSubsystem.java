package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tools.Trajectories;
import org.stealthrobotics.library.StealthSubsystem;

public class HoodSubsystem extends StealthSubsystem {
    private final Servo hoodServo;

    // TODO:determine 0 position
    public static final double MIN_POSITION = 0;
    //Pos    Deg
    // 0.05 - 0
    // 0.10 - 2.8
    // 0.20 - 8.0
    // 0.30 - 13.5
    // 0.40 - 18.8
    // 0.50 - 24.0
    // 0.60 - 28.8
    public static final double MAX_POSITION = .60; // 28.8

    private InterpLUT rangeToHoodAngle;
    public static final double MIN_DISTANCE = 30;
    public static final double MAX_DISTANCE = 70;
    public double angleOverride = 0;
    private static final double DEFAULT_POSITION = 0.5;

    public HoodSubsystem(HardwareMap map) {
        hoodServo = map.get(Servo.class, "hoodServo");
        hoodServo.setPosition(DEFAULT_POSITION);
        rangeToHoodAngle= Trajectories.createHoodTable();
    }

    /**
     * Gets the current angle of the hood by converting the servo position to an angle.
     * @return The current angle of the hood in degrees.
     */
    public double getAngle (){
        double x = hoodServo.getPosition();
        return -2.52577 + 51.02454 * x + 13.54213 * Math.pow(x, 2) - 19.22261 * Math.pow(x, 3);
    }

    /**
     * Gets the position of the hood by converting the angle to a servo position.
     * @param x The angle of the hood in degrees.
     * @return The position of the hood as a servo position.
     */
    public double degreesToPosition (double x) {
        return 0.04889851 + 0.01919993 * x - 0.00007917742 * Math.pow(x, 2) + 0.000002657166 * Math.pow(x, 3);
    }

    /**
     * Sets the position of the hood servo.
     * @param pos The position of the hood as a servo position.
     */
    public void setPosition(double pos) {
        pos += angleOverride;
        pos = Math.clamp(pos, MIN_POSITION, MAX_POSITION);
        hoodServo.setPosition(pos);
    }

    public void setHoodAngle(double distance) {
        double angle = getHoodAngleFromRange(distance);

        setPosition(degreesToPosition(angle));
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
    public double getPosition() {
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
