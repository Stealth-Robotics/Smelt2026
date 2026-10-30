package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "OtherNearRedAuto")
public class OtherNearRedAuto extends BioBuzzAutos
{
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.8844, 8.2077, 90);
    private final Pose alignFlower = poseFactory.of(29.6241, 47, 86.73);
    private final Pose alignFlowerControl1 = poseFactory.of(28.318, 24.1395, 0);
    private final Pose harvest = poseFactory.of(14, 47, 0);
    private final Pose postHarvest = poseFactory.of(18, 47, 0);
    private final Pose goShootToo = poseFactory.of(45.7115, 123.7349, 173.5971);
    private final Pose goShootTooControl1 = poseFactory.of(4.0117, 128.4145, 0);
    private final Pose point5 = poseFactory.of(47.4095, 120.1535, 115.3667);
    private final Pose parkIt = poseFactory.of(11.8202, 119.4826, -178.9201);

    public Path alignFlower() {
        return Paths.curve(start, alignFlowerControl1, alignFlower).tangent();
    }

    public Path harvest() {
        return Paths.line(alignFlower, harvest).reverseTangent();
    }

    public Path postHarvest() {
        return Paths.line(harvest, postHarvest).tangent();
    }

    public Path goShootToo() {
        return Paths.curve(postHarvest, goShootTooControl1, goShootToo).reverseTangent();
    }

    public Path path5() {
        return Paths.line(goShootToo, point5).reverseTangent();
    }

    public Path parkIt() {
        return Paths.line(point5, parkIt).tangent();
    }
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                shootSequence(),
                drive.followPathCommand(alignFlower()),
                robot.getHarvesterSubsystem().HarvDown(),
                intake.startAutoIntakeCommand(true),
                drive.followPathCommand(harvest()),
                waitFor(3000),
                drive.followPathCommand(postHarvest()),
                intake.setAutoIntakeCommand(false),
                robot.getHarvesterSubsystem().HarvUp(),
                intake.stopAutoIntakeCommand(),
                drive.followPathCommand(goShootToo()),
                drive.followPathCommand(path5()),
                shootSequence(),
                drive.followPathCommand(parkIt())
        );
    }
}
