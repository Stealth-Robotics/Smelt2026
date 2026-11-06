package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;

public class HoodSubsystem extends StealthSubsystem {
    private final Servo hoodServo;

    // TODO:determine 0 position
    private static final double MIN_POSITION = 0.058;
    //noted numbers
    // 0.1 - 2.2
    // 0.2 - 8.0
    // 0.3 - 13.4
    // 0.4 - 18.4
    // 0.5 - 24.4
    // 0.542 - 28.8
    private static final double MAX_POSITION = 0.542; // 28.8

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

    /**
     * Gets the current angle of the hood by converting the servo position to an angle.
     * @return The current angle of the hood in degrees.
     */
    public double getAngle (){
        double x = hoodServo.getPosition();
        return -1.743168 + 14.06975 * x + 341.4512 * Math.pow(x, 2) - 1031.662 * Math.pow(x, 3) + 1005.784 * Math.pow(x, 4);
    }

    /**
     * Gets the position of the hood by converting the angle to a servo position.
     * @param x The angle of the hood in degrees.
     * @return The position of the hood as a servo position.
     */
    public double degreesToPosition (double x){
        return 0.05841016 + 0.01935597 * x - 0.0004159367 * Math.pow(x, 2) + 0.00003531975 * Math.pow(x, 3) - 8.326964e-7 * Math.pow(x, 4);
    }

    private void buildRangeTable() {
        // TODO: make these angles for easier readability and move this to a tools class Trajectories or something

        rangeToHoodAngle = new InterpLUT();
        rangeToHoodAngle.add(MIN_DISTANCE - 0.1, 0.23);
        rangeToHoodAngle.add(40, 0.26);
        rangeToHoodAngle.add(48, 0.39);
        rangeToHoodAngle.add(50, .392);
        rangeToHoodAngle.add(55, .41);
        rangeToHoodAngle.add(57,0.45);
        rangeToHoodAngle.add(60, .49);
        rangeToHoodAngle.add(62, 0.500);
        rangeToHoodAngle.add(MAX_DISTANCE + 0.1, MAX_POSITION);

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
