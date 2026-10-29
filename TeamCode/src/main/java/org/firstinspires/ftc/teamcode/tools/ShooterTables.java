package org.firstinspires.ftc.teamcode.tools;

import com.arcrobotics.ftclib.util.InterpLUT;

public class ShooterTables {

    private InterpLUT rangeToRpm;
    private static final double MIN_DISTANCE = 35;
    private static final double MAX_DISTANCE = 75;

    public void buildShooterTable() {
        rangeToRpm = new InterpLUT();
        rangeToRpm.add(MIN_DISTANCE - 0.1, 3550);
        rangeToRpm.add(40.0, 3600);
        rangeToRpm.add(45.0, 3620);
        rangeToRpm.add(50.0, 3610);
        rangeToRpm.add(55.0, 3700);
        rangeToRpm.add(69.0, 3950);
        rangeToRpm.add(MAX_DISTANCE + 0.1, 4000);
        rangeToRpm.createLUT();
    }



}
