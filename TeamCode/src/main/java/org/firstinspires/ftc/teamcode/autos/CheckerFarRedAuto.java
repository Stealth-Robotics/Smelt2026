package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.stealthrobotics.library.commands.IfCommand;

@Autonomous(name = "Checker Far Red Auto", group = "BLUE")
public class CheckerFarRedAuto extends BioBuzzAutos
{
    private final PoseFactory poseFactory = PoseFactory.degrees()
            .rotateAround(FIELD_CENTER,180);

    private final Pose start = poseFactory.of(61.2425, 133.2425, 90);
    private final Pose goShoot = poseFactory.of(46.8609, 120.5233, 110);
    private final Pose goShootControl1 = poseFactory.of(62.406, 123.6543, 0);
    private final Pose harvest = poseFactory.of(46.8796, 129.8951, 89.8855);
    private final Pose checkStart = poseFactory.of(46.8796, 129.8951, 90);
    private final Pose check = poseFactory.of(46.86, 120.52, 110);
    private final Pose goParkForTrue = poseFactory.of(10.3175, 120.6879, -0.2633);
    private final Pose irrelevantPath = poseFactory.of(46.86, 120.52, 179.7367);
    private final Pose goShootOtherSide = poseFactory.of(46.7685, 15.7031, 62.07);
    private final Pose goShootOtherSideControl1 = poseFactory.of(79.6566, 77.7392, 0);
    private final Pose shootOtherSide = poseFactory.of(49.2259, 22.8459, -108.9849);
    private final Pose goParkForFalse = poseFactory.of(12.1649, 92.1592, -90.1016);
    private final Pose goParkForFalseControl1 = poseFactory.of(12.1079, 60.006, 0);

    public Path goShoot() {
        return Paths.curve(start, goShootControl1, goShoot).linear(start, goShoot);
    }

    public Path harvest() {
        return Paths.line(goShoot, harvest).tangent();
    }

    public Path check() {
        return Paths.line(checkStart, check).linear(checkStart, check);
    }

    public Path goParkForTrue() {
        return Paths.line(check, goParkForTrue).reverseTangent();
    }

    public Path irrelevantPath() {
        return Paths.line(goParkForTrue, irrelevantPath).reverseTangent();
    }

    public Path goShootOtherSide() {
        return Paths.curve(irrelevantPath, goShootOtherSideControl1, goShootOtherSide).reverseTangent();
    }

    public Path shootOtherSide() {
        return Paths.line(goShootOtherSide, shootOtherSide).reverseTangent();
    }

    public Path goParkForFalse() {
        return Paths.curve(shootOtherSide, goParkForFalseControl1, goParkForFalse).reverseTangent();
    }


    @Override
    public void initialize(){
        super.initialize();
        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                drive.followPathCmd(goShoot()), //Path goShoot
                waitCmd(10000),             //Wait 10 seconds
                new IfCommand(this::getTagSeen,
                        // Tags Seen == TRUE
                        new SequentialCommandGroup
                                (
                                        shooter.startShootCmd(51),
                                        robot.getHarvesterSubsystem().HarvUp(),
                                        drive.followPathCmd(harvest()),
                                        drive.followPathCmd(check()),
                                        shootSequenceCmd(51),
                                        robot.getHarvesterSubsystem().HarvDown(),
                                        drive.followPathCmd(goParkForTrue())
                                ),
                        // Tags Seen == FALSE
                        new SequentialCommandGroup
                                (
                                        drive.followPathCmd(goShootOtherSide()),
                                        drive.followPathCmd(shootOtherSide()),
                                        shootSequenceCmd(51),
                                        drive.followPathCmd(goParkForFalse())
                                )
                )

        );
    }
}
