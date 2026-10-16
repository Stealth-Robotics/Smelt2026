package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class DriveSubsystem extends StealthSubsystem
{
    private final Follower follower;
    private final Telemetry telemetry = StealthOpMode.telemetry;

    private static final Pose bluePose = new Pose(FieldConstants.FIELD_X_MAX - FieldConstants.ROBOT_CENTER_X, FieldConstants.ROBOT_CENTER_Y, FieldConstants.ROBOT_BLUE_HEADING);
    private static final Pose redPose = new Pose(FieldConstants.ROBOT_CENTER_X, FieldConstants.ROBOT_CENTER_Y, FieldConstants.ROBOT_RED_HEADING);

    public DriveSubsystem(HardwareMap map){
        follower = Constants.create(map);
        follower.setPose(Alliance.isBlue()?bluePose:redPose);
        follower.localizer.reset();
    }

    @Override
    public void periodic(){
        follower.update();
        Pose pose = follower.pose();
//        telemetry.addData("X",pose.x());
//        telemetry.addData("Y",pose.y());
//        telemetry.addData("Heading",Math.toDegrees(pose.heading()));
    }

    public Follower getFollower(){
        return follower;
    }

    public void resetOdometry(boolean resetPose) {
        Pose basePose = resetPose ? (Alliance.isBlue() ? bluePose : redPose) : follower.pose();

        double heading = Alliance.isBlue() ? FieldConstants.ROBOT_BLUE_HEADING : FieldConstants.ROBOT_RED_HEADING;
        follower.setPose(new Pose(basePose.x(), basePose.y(), heading));
    }
}
