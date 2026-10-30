package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//sideways shuffling path
@Autonomous(name = "OtherNearRedAuto")
public class OtherNearRedAuto extends BioBuzzAutos
{

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.8844, 8.2077, 270);
    private final Pose point1 = poseFactory.of(13.8384, 46.9578, 180);
    private final Pose point2Start = poseFactory.of(13.8384, 46.9578, 180);
    private final Pose point2 = poseFactory.of(56.5122, 117.9192, 90);
    private final Pose point2Control1 = poseFactory.of(23.3284, 121.7679, 0);
    private final Pose point3Start = poseFactory.of(56.5122, 117.9192, 90);
    private final Pose point3 = poseFactory.of(8.9123, 115.5362, 180);

    public Path goShoot() {
        return Paths.line(start, point1).linear(start, point1);
    }

    public Path goToIntake() {
        return Paths.line(point2Start, point2).linear (point2Start, point2);
    }

    public Path intake() {
        return Paths.line(point2Control1, point3Start).linear(point2Control1, point3Start);
    }

    public Path goShoot2() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public void initialize() {
        super.initialize();
        follower.setPose(start);
    }
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                drive.followPathCommand(goShoot()),
                shootSequence(),
                drive.followPathCommand(goToIntake()),
                intake.startAutoIntakeCommand(true),
                drive.followPathCommand(intake()),
                intake.stopAutoIntakeCommand(),
                drive.followPathCommand(goShoot2())
        );
    }
}
