package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class DriveSubsystem extends StealthSubsystem
{
    private final Follower follower;
    private final Telemetry telemetry = StealthOpMode.telemetry;

    private static final Pose bluePose = new Pose(0, 0, 180);
    private static final Pose redPose = new Pose(0, 0, 180);

    public DriveSubsystem(HardwareMap map){

        follower = Constants.getFollower(map);
        follower.setPose(Alliance.isBlue()?bluePose:redPose);
        follower.localizer.reset();
    }
    @Override
    public void periodic(){
        follower.update();
        Pose pose = follower.pose();
        telemetry.addData("X",pose.x());
        telemetry.addData("Y",pose.y());
        telemetry.addData("Heading",Math.toDegrees(pose.heading()));
        telemetry.addData("Aliance: ", Alliance.get());
    }
    public Follower getFollower(){
        return follower;
    }

    public void resetOdometry(){
        follower.localizer.reset();

        Pose pose = follower.pose();
        if (Alliance.isBlue()) {
            pose = new Pose(pose.x(), pose.y(), 180);
        } else {
            pose = new Pose(pose.x(), pose.y(), 0);
        }
        follower.setPose(pose);
    }
}
