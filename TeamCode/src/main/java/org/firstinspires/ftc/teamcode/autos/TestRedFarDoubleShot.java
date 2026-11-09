package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.firstinspires.ftc.teamcode.tools.Targets;
import org.stealthrobotics.library.commands.IfCommand;

@Autonomous(name ="Test Far RED", group = "Test")
public class TestRedFarDoubleShot extends BioBuzzAutos{

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.9032, FieldConstants.FIELD_Y_MAX - FieldConstants.ROBOT_CENTER_Y, 90);
    private final Pose readyShoot = poseFactory.of(46.75, 118.75, 90);
    private final Pose readyShootControl1 = poseFactory.of(61.7869, 118.5712, 0);
    private final Pose harvest = poseFactory.of(46.75, 129, 90);
    private final Pose shootToo = poseFactory.of(46.75, 118.75, 110);
    private final Pose shootTooSegment1Heading = poseFactory.of(46.75, 118.75, 90);
    private final Pose parkIt = poseFactory.of(13.75, 118.75, 180);



    public Path readyShoot() {
        return Paths.curve(start, readyShootControl1, readyShoot)
                .heading(Interpolator.piecewise()
                        .until(0.3, Interpolator.constant(readyShoot))
                        .until(1, Interpolator.facingPoint(Targets.getTarget(readyShoot)).reverse()));

    }

    public Path harvest() {
        return Paths.line(readyShoot, harvest).constant(harvest);
    }

    public Path shootToo() {
        return Paths.line(harvest, shootToo)
                .heading(Interpolator.piecewise()
                        .until(0.5, Interpolator.constant(shootTooSegment1Heading))
                        .until(1, Interpolator.facingPoint(Targets.getTarget(shootToo)).reverse()));
    }

    public Path parkIt() {
        return Paths.line(shootToo, parkIt).tangent();
    }

    public void initialize() {
        super.initialize();
        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                shooter.startFloatCmd(), // TODO: move to below a 4 or so second wait
                drive.followPathCmd(readyShoot()),
                harvester.harvDownCmd(),
                waitForTagsCmd(10000),
                new IfCommand(this::getTagSeen,
                        // Tags Seen == TRUE
                        new SequentialCommandGroup(
                                shootSequenceCmd(),
                                shooter.stopFloatCmd(),
                                startHarvestingCmd(),
                                drive.followPathCmd(harvest()),
                                waitCmd(2000),
                                shooter.startFloatCmd(),
                                drive.followPathCmd(shootToo()),
                                stopHarvestingCmd(),
                                shootSequenceCmd(),
                                shooter.stopFloatCmd(),
                                drive.followPathCmd(parkIt())
                        ),
                        // FALSE NO TAGS SEEN GO AND SHOOT OTHER SIDE
                        // TODO:finish this
                        new SequentialCommandGroup(
                                drive.followPathCmd(parkIt())
                        ))
                );
    }
}
