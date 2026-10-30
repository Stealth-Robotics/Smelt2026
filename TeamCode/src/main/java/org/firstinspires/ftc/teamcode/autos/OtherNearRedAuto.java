package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Other Near Red Auto")
public class OtherNearRedAuto extends BioBuzzAutos
{
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.8844, 8.2077, 90);
    private final Pose moveUp = poseFactory.of(56.838, 12.3968, -89.3649);
    private final Pose alignFlower = poseFactory.of(29.6241, 47, 86.73);
    private final Pose alignFlowerControl1 = poseFactory.of(28.318, 24.1395, 0);
    private final Pose harvest = poseFactory.of(14, 47, 180);
    private final Pose postHarvest = poseFactory.of(18, 47, 180);
    private final Pose goShootToo = poseFactory.of(32.626, 120.2039, 65.2955);
    private final Pose goShootTooControl1 = poseFactory.of(18.3434, 89.158, 0);
    private final Pose aimforshootStart = poseFactory.of(32.626, 120.2039, 90);
    private final Pose aimForShoot = poseFactory.of(40.3475, 126.177, 120);
    private final Pose parkIt = poseFactory.of(11.8202, 119.4826, -166.7936);

    public Path moveUp() {
        return Paths.line(start, moveUp).reverseTangent();
    }

    public Path alignFlower() {
        return Paths.curve(moveUp, alignFlowerControl1, alignFlower).tangent();
    }

    public Path harvest() {
        return Paths.line(alignFlower, harvest).tangent();
    }

    public Path postHarvest() {
        return Paths.line(harvest, postHarvest).reverseTangent();
    }

    public Path goShootToo() {
        return Paths.curve(postHarvest, goShootTooControl1, goShootToo).tangent();
    }

    public Path aimforshoot() {
        return Paths.line(aimforshootStart, aimForShoot).linear(aimforshootStart, aimForShoot);
    }

    public Path parkIt() {
        return Paths.line(aimForShoot, parkIt).tangent();
    }

    @Override
        public void initialize(){
        super.initialize();
        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                shootSequence(59),
                drive.followPathCommand(moveUp()),
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
                drive.followPathCommand(aimforshoot()),
                shootSequence(),
                drive.followPathCommand(parkIt())
        );
    }
}
