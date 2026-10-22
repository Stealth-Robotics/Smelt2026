package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tools.Pipeline;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.lang.reflect.Parameter;
import java.util.ArrayList;
import java.util.List;

public class CameraSubsystem extends StealthSubsystem {
    private final Limelight3A limelight;
    private Telemetry telemetry = StealthOpMode.telemetry;

    //i'm not sure what i'm supposed to do for the goal blue/red ids? I don't know if those have changed
    //or if it's affected by our whole april tagging system ordeal
    //will update when i DO know
    //thankfully as of 10/21 april tag ids and pipeline are set!!!
    //still not sure what to do for the goal blue/red ids LOL
    public CameraSubsystem (HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(250);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        telemetry.addData("Pipeline", getPipeline());


    }

    public Pipeline getPipeline() {
        return Pipeline.values()[limelight.getStatus().getPipelineIndex()];
    }

    public LLResult getLastResult() {
        return limelight.getLatestResult();
    }

    public void setPipeline(Pipeline pipeline) {
        if (pipeline == getPipeline()) {
            return;
        }
        limelight.pipelineSwitch(pipeline.id);

    }
    private List<Integer> getTagIds(LLResult result) {
        List<Integer> tags = new ArrayList<>();
        if (result == null || !result.isValid()) {
            return tags;
        }

    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        for (int i = 0; i < fiducials.size(); i++) {
            tags.add(fiducials.get(i).getFiducialId());
        }

        return tags;
    }
}
