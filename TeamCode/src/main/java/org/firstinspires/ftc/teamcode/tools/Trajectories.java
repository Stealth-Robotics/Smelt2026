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
        rangeToHoodAngle.add(MIN_DISTANCE - 0.1, 4.928);
        rangeToHoodAngle.add(40, 8.920);
        rangeToHoodAngle.add(45, 11.281);
        rangeToHoodAngle.add(50, 14.949);
        rangeToHoodAngle.add(55, 19.558);
        rangeToHoodAngle.add(56.60, 19.99);
        rangeToHoodAngle.add(60, 23.392);
        rangeToHoodAngle.add(65, 23.468);
        rangeToHoodAngle.add(70, 26.977);
        rangeToHoodAngle.add(75, 27.781);
        rangeToHoodAngle.add(80.73, 28.24);
        rangeToHoodAngle.add(85.72, 28.475);
        rangeToHoodAngle.add(92, 28.710);
        rangeToHoodAngle.add(MAX_DISTANCE + 0.1, 28.8);
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
        rangeToRpm.add(MIN_DISTANCE - 0.1, 2950);
        rangeToRpm.add(40.0, 2990);
        rangeToRpm.add(45.0, 3025);
        rangeToRpm.add(50.0, 3159);
        rangeToRpm.add(55.0, 3135);
        rangeToRpm.add(56.60, 3149);
        rangeToRpm.add(60.0, 3120);
        rangeToRpm.add(65.0, 3153);
        rangeToRpm.add(70.0, 3212);
        rangeToRpm.add(75.0, 3315);
        rangeToRpm.add(80.73, 3411);
        rangeToRpm.add(85.72, 3539);
        rangeToRpm.add(92.0, 3615);
        rangeToRpm.add(MAX_DISTANCE + 0.1, 3675);
        rangeToRpm.createLUT();
        return rangeToRpm;
    }
}
