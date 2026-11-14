package org.firstinspires.ftc.teamcode.tools;

import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.DmShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;

public class Trajectories {

    /**
     * Creates a new Hood interpolation table
     * @return The hood table
     */
    public static InterpLUT createHoodTable() {
        final double MIN_DISTANCE = HoodSubsystem.MIN_DISTANCE;
        final double MAX_DISTANCE = HoodSubsystem.MAX_DISTANCE;

        InterpLUT rangeToHoodAngle = new InterpLUT();
        rangeToHoodAngle.add(MIN_DISTANCE - 0.1, 10.00);
        rangeToHoodAngle.add(40, 13.006);
        rangeToHoodAngle.add(48, 16.334);
        rangeToHoodAngle.add(50, 18.239);
        rangeToHoodAngle.add(55, 18.741);
        rangeToHoodAngle.add(57, 20.965);
        rangeToHoodAngle.add(60, 24.385);
        rangeToHoodAngle.add(62, 24.142);
        rangeToHoodAngle.add(MAX_DISTANCE + 0.1, 28.724);
        rangeToHoodAngle.createLUT();
        return rangeToHoodAngle;
    }

    /**
     * Creates a new shooter interpolation table
     * @return The shooter table
     */
    public static InterpLUT createShooterTable() {
        final double MIN_DISTANCE = DmShooterSubsystem.MIN_DISTANCE;
        final double MAX_DISTANCE = DmShooterSubsystem.MAX_DISTANCE;
        InterpLUT rangeToRpm = new InterpLUT();
        rangeToRpm.add(MIN_DISTANCE - 0.1, 3050);
        rangeToRpm.add(40.0, 3190);
        rangeToRpm.add(45.0, 3210);
        rangeToRpm.add(50.0, 3214);
        rangeToRpm.add(55.0, 3230);
        rangeToRpm.add(60.0, 3290);
        rangeToRpm.add(61.0, 3300);
        rangeToRpm.add(62.0, 3290);
        rangeToRpm.add(69.0, 3370);
        rangeToRpm.add(MAX_DISTANCE + 0.1, 3440);
        rangeToRpm.createLUT();
        return rangeToRpm;
    }
}
