package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

public class CameraSubsystem extends StealthSubsystem {
    private final Limelight3A limelight;
    //i'm not sure what i'm supposed to do for the goal blue/red ids? I don't know if those have changed
    //or if it's affected by our whole april tagging system ordeal
    //will update when i DO know
    public CameraSubsystem (HardwareMap hardwareMap){
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.setPollRateHz(250);
    limelight.pipelineSwitch(0);
    limelight.start();


        //setting up limelight. wip ignore the mess ^.^
        //are vision and camera subsystem the same thing...? idk lol prolly are


    }


}
