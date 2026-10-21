package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.tools.AutoToTeleOpStorage;
import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class DriveSubsystem extends StealthSubsystem
{
    private final Follower follower;
    private final Telemetry telemetry = StealthOpMode.telemetry;

    private static final Pose bluePose = new Pose(FieldConstants.FIELD_X_MAX - FieldConstants.ROBOT_CENTER_X, FieldConstants.ROBOT_CENTER_Y, FieldConstants.ROBOT_BLUE_HEADING);

    // TODO: change this to a better start point note that robot right now default to back corner of red vs front corner
    private static final Pose redPose = new Pose(FieldConstants.ROBOT_CENTER_X, FieldConstants.FIELD_Y_MAX - FieldConstants.ROBOT_CENTER_Y, FieldConstants.ROBOT_RED_HEADING);


    public DriveSubsystem(HardwareMap map){
        follower = Constants.create(map);
        if (AutoToTeleOpStorage.isPoseValid()) {
            follower.setPose(AutoToTeleOpStorage.getPose());
        }else {
            follower.setPose(Alliance.isBlue() ? bluePose : redPose);
        }
        follower.localizer.reset();
    }


    @Override
    public void periodic(){
        follower.update();
        Pose pose = follower.pose();
        telemetry.addData("PP X",pose.x());
        telemetry.addData("PP Y",pose.y());
        telemetry.addData("PP Heading",Math.toDegrees(pose.heading()));
        telemetry.addData("Distance to end", follower.distanceToEndpoint());
    }

    public Follower getFollower(){
        return follower;
    }

    public void resetOdometry(boolean resetPose) {
        Pose basePose = resetPose ? (Alliance.isBlue() ? bluePose : redPose) : follower.pose();

        double heading = Alliance.isBlue() ? FieldConstants.ROBOT_BLUE_HEADING : FieldConstants.ROBOT_RED_HEADING;
        follower.setPose(new Pose(basePose.x(), basePose.y(), heading));
    }

    public Command resetOdometryCommand(boolean resetPose) {
        return new InstantCommand(() -> resetOdometry(resetPose), this);
    }

    public Command resetOdometryCommand() {
        return resetOdometryCommand(false);
    }

    /**
     * Follows a path with no timeout and no hold
     * @param path Pedro path to follow
     * @return Command
     */
    public Command followPathCommand(Path path){
        return followPathCommand(path, false);
    }

    /**
     * Follows a path with then optionally holds at the end point
     * @param path Path to follow
     * @param holdPoint true for hold at end otherwise false
     * @return Command
     */
    public Command followPathCommand(Path path, boolean holdPoint){
        return new InstantCommand(() -> follower.follow(path), this)
                .andThen(new WaitUntilCommand(follower::atParametricEnd)
                        .whenFinished(() -> { if (holdPoint) follower.hold(path.endPose()); }));
    }

    /**
     * Follows a path with a timeout and optionally holds at the end point
     * @param path Path to follow
     * @param timeout number of milliseconds to wait for path to finish
     * @param holdPoint true for hold at end otherwise false
     * @return Command
     */
    public Command followPathCommand(Path path, long timeout, boolean holdPoint){
        long endtime = System.currentTimeMillis() + timeout;
        return new InstantCommand(() -> follower.follow(path), this)
                .andThen(new WaitUntilCommand(() -> (follower.atParametricEnd() || System.currentTimeMillis() > endtime)))
                .whenFinished(() -> { if (holdPoint) follower.hold(path.endPose()); });
    }
}


