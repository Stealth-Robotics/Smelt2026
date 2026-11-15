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
        rangeToHoodAngle.add(MIN_DISTANCE - 0.1, 6.00);
        rangeToHoodAngle.add(40, 10.006);
        rangeToHoodAngle.add(45, 12.37);
        rangeToHoodAngle.add(50, 15.93);
        rangeToHoodAngle.add(55, 20.500);
        rangeToHoodAngle.add(60, 24.385);
        rangeToHoodAngle.add(65, 24.460);
        rangeToHoodAngle.add(70, 27.926);

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
        rangeToRpm.add(MIN_DISTANCE - 0.1, 3150);
        rangeToRpm.add(40.0, 3190);
        rangeToRpm.add(45.0, 3225);
        rangeToRpm.add(50.0, 3359);
        rangeToRpm.add(55.0, 3330);
        rangeToRpm.add(60.0, 3320);
        rangeToRpm.add(65.0, 3353);
        rangeToRpm.add(70.0, 3412);
        rangeToRpm.add(MAX_DISTANCE + 0.1, 3440);
        rangeToRpm.createLUT();
        return rangeToRpm;
    }
}
