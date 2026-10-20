package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

public class CameraSubsystem extends StealthSubsystem {
    Limelight3A limelight;
    public CameraSubsystem (HardwareMap hardwareMap){
limelight = hardwareMap.get(Limelight3A.class, "limelight");
limelight.setPollRateHz(250);
limelight.pipelineSwitch(0);
limelight.start();
        //setting up limelight. wip ignore the mess ^.^
    }
}
