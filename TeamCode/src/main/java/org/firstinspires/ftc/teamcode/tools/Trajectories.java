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
        rangeToHoodAngle.add(50, 14.849);
        rangeToHoodAngle.add(55, 19.457);
        rangeToHoodAngle.add(60, 23.392);
        rangeToHoodAngle.add(65, 23.468);
        rangeToHoodAngle.add(70, 26.977);

        rangeToHoodAngle.add(MAX_DISTANCE + 0.1, 27.781);
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
        rangeToRpm.add(MIN_DISTANCE - 0.1, 3100);
        rangeToRpm.add(40.0, 3140);
        rangeToRpm.add(45.0, 3175);
        rangeToRpm.add(50.0, 3309);
        rangeToRpm.add(55.0, 3280);
        rangeToRpm.add(60.0, 3270);
        rangeToRpm.add(65.0, 3303);
        rangeToRpm.add(70.0, 3362);
        rangeToRpm.add(MAX_DISTANCE + 0.1, 3390);
        rangeToRpm.createLUT();
        return rangeToRpm;
    }
}
