package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class DriveSubsystem extends StealthSubsystem
{
    private final Follower follower;
    private final Telemetry telemetry = StealthOpMode.telemetry;
    public DriveSubsystem(HardwareMap map){
        follower = Constants.getFollower(map);
    }
    @Override
    public void periodic(){
        follower.update();
        Pose pose = follower.pose();
        telemetry.addData("X",pose.x());
        telemetry.addData("Y",pose.y());
        telemetry.addData("Heading",Math.toDegrees(pose.heading()));
    }
    public Follower getFollower(){
        return follower;
    }
}
