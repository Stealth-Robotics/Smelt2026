package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous (name = "testauto")
public class testauto extends BiobuzzAutos {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56, 36, 90);

    private RobotSystem robot;

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public void initialize() {
        super.initialize();
        follower.setPose(start);
    }
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(drive.followPathCommand(path1()));
    }
}