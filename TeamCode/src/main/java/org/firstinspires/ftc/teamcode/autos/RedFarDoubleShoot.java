package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.config.Modifier;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;


@Autonomous (name = "Red Far Double Shoot park", group = "RED")
public class RedFarDoubleShoot extends BioBuzzAutos {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(62, 132.5, 270);
    private final Pose goShoot = poseFactory.of(62, 130, 260);
    private final Pose afterFirstShotStart = poseFactory.of(62, 130, 260);
    private final Pose afterFirstShot = poseFactory.of(48, 126, 270);
    private final Pose collectFromFlower = poseFactory.of(48, 132, -90);
    private final Pose shootAgainStart = poseFactory.of(48, 132, 270);
    private final Pose shootAgain = poseFactory.of(48.1615, 124.6204, 280);
    private final Pose park = poseFactory.of(13.432, 121.5578, 178.8799);
    private final Pose parkControl1 = poseFactory.of(40.5061, 121.0095, 0);

    public Path goShoot() {
        return Paths.line(start, goShoot).linear(start, goShoot);
    }

    public Path afterFirstShot() {
        return Paths.line(afterFirstShotStart, afterFirstShot).linear(afterFirstShotStart, afterFirstShot);
    }

    // Example of controlling speed
    public Path collectFromFlower() {
        return Paths.line(afterFirstShot, collectFromFlower).reverseTangent()
                .with(
                    Constants.foresightConfig.maxPathSpeed.at(0.4)
        );
    }

    public Path shootAgain() {
        return Paths.line(shootAgainStart, shootAgain).linear(shootAgainStart, shootAgain);
    }

    public Path park() {
        return Paths.curve(shootAgain, parkControl1, park).tangent();
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
                drive.followPathCommand(afterFirstShot()),
                intake.startAutoIntakeCommand(),
                drive.followPathCommand(collectFromFlower()),
                waitFor(4000),
                intake.stopAutoIntakeCommand(),
                drive.followPathCommand(shootAgain()),
                shootSequence()
                //,drive.followPathCommand(park())
        );
    }
}
