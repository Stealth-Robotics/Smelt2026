package org.firstinspires.ftc.teamcode.tools;

import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.DmShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;

public class Trajectories {
    public static InterpLUT createHoodTable() {
        final double MIN_DISTANCE = HoodSubsystem.MIN_DISTANCE;
        final double MAX_DISTANCE = HoodSubsystem.MAX_DISTANCE;
        final double MAX_POSITION = HoodSubsystem.MAX_POSITION;

        InterpLUT rangeToHoodAngle = new InterpLUT();
        rangeToHoodAngle.add(MIN_DISTANCE - 0.1, 0.23);
        rangeToHoodAngle.add(40, 0.29);
        rangeToHoodAngle.add(48, 0.39);
        rangeToHoodAngle.add(50, .392);
        rangeToHoodAngle.add(55, .41);
        rangeToHoodAngle.add(57,0.45);
        rangeToHoodAngle.add(60, .49);
        rangeToHoodAngle.add(62, 0.500);
        rangeToHoodAngle.add(MAX_DISTANCE + 0.1, MAX_POSITION);
        rangeToHoodAngle.createLUT();
        return rangeToHoodAngle;
    }
    public static InterpLUT createShooterTable() {
        final double MIN_DISTANCE = DmShooterSubsystem.MIN_DISTANCE;
        final double MAX_DISTANCE = DmShooterSubsystem.MAX_DISTANCE;
        InterpLUT rangeToRpm = new InterpLUT();
        rangeToRpm = new InterpLUT();
        rangeToRpm.add(MIN_DISTANCE - 0.1, 3550);
        rangeToRpm.add(40.0, 3600);
        rangeToRpm.add(45.0, 3620);
        rangeToRpm.add(50.0, 3610);
        rangeToRpm.add(55.0, 3625);
        rangeToRpm.add(59.0, 3650);
        rangeToRpm.add(61.0, 3680);
        rangeToRpm.add(62.0, 3700);
        rangeToRpm.add(69.0, 3900);
        rangeToRpm.add(MAX_DISTANCE + 0.1, 4000);
        rangeToRpm.createLUT();
        return rangeToRpm;
    }
}
