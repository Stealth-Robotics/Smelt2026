package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.stealthrobotics.library.commands.IfCommand;

// This show how to build a conditional path auto.
// If tags are seen shoot collect shoot, home. if no tags seen after Xms go to the other side and shoot.
// This is also showing how to flip the field. for blue side. I see some teams rotate around 72
public class FarDoubleShoot extends BioBuzzAutos {

    private final Pose start;
    private final Pose goShoot;
    private final Pose afterFirstShot;
    private final Pose collectFromFlower;
    private final Pose shootAgainStart;
    private final Pose shootAgain;
    private final Pose park;
    private final Pose parkControl1;
    // failed to tip

    private final Pose noHive;
    private final Pose shootOtherSide;
    private final Pose goBackForPark;
    private final Pose goBackForParkControl1;

    public FarDoubleShoot(boolean mirror) {
        super();
        PoseFactory poseFactory;
        if (mirror) {
            poseFactory = PoseFactory.degrees().mirrorX(FieldConstants.FIELD_X_MAX / 2);
        } else {
            poseFactory = PoseFactory.degrees();
        }

        start = poseFactory.of(60, 132, 90);
        goShoot = poseFactory.of(60, 121, 88);
        afterFirstShot = poseFactory.of(47.5, 120, 90);
        collectFromFlower = poseFactory.of(47.5, 132, 90);
        shootAgainStart = poseFactory.of(47.5, 132, 90);
        shootAgain = poseFactory.of(49, 116, 109);
        park = poseFactory.of(15.0343, 123.9935, -178.6281);
        parkControl1 = poseFactory.of(40.7941, 124.6549, 0);

        noHive = poseFactory.of(59, 114, 270);
        shootOtherSide = poseFactory.of(59, 22, -90);
        goBackForPark = poseFactory.of(48, 121.5, -10);
        goBackForParkControl1 = poseFactory.of(65, 119, 0);
    }

    public Path goShoot() {
        return Paths.line(start, goShoot).linear(start, goShoot);
    }

    public Path afterFirstShot() {
        return Paths.line(goShoot, afterFirstShot).constant(afterFirstShot);
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

    public Path noHive() {
        return Paths.line(goShoot, noHive).linear(start, noHive);
    }

    public Path shootOtherSide() {
        return Paths.line(noHive, shootOtherSide).reverseTangent();
    }

    public Path goBackForPark() {
        return Paths.curve(shootOtherSide, goBackForParkControl1, goBackForPark).tangent();
    }

    public void initialize() {
        super.initialize();
        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                drive.followPathCmd(goShoot()),
                waitForTagsCmd(8000),
                // If we time out on tags do something different
                new IfCommand(this::getTagSeen,
                    // Tags Seen == TRUE
                    new SequentialCommandGroup(
                        //shootSequence(),
                          shooter.startShootCmd(40),
//                        drive.followPathCommand(afterFirstShot()),
//                        intake.startAutoIntakeCommand(true),
//                        drive.followPathCommand(collectFromFlower()),
//                        waitFor(4000),
//                        intake.stopAutoIntakeCommand(),
//                        drive.followPathCommand(shootAgain()),
//                        shootSequence(),
//                    // tags seen == FALSE
                    new SequentialCommandGroup(
//                            drive.followPathCommand(noHive()),
//                            drive.followPathCommand(shootOtherSide()),
//                            shootSequence(),
//                            drive.followPathCommand(goBackForPark())
                    )))
                //,drive.followPathCommand(park())
        );
    }
    @Autonomous (name = "Red Far Double Shoot park", group = "RED")
    public static class RedFarDoubleShoot extends FarDoubleShoot {
        public RedFarDoubleShoot() {
            super(false);
        }
    }

    @Autonomous (name = "Blue Far Double Shoot park", group = "BLUE")
    public static class BlueFarDoubleShoot extends FarDoubleShoot {
        public BlueFarDoubleShoot() {
            super(true);
        }
    }
}
