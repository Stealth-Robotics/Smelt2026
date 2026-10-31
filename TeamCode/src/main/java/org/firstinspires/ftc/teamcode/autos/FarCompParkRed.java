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
import org.stealthrobotics.library.commands.IfCommand;

@Autonomous(name = "Compat Red shoot flower or park", group = "RED")
public class FarCompParkRed extends BioBuzzAutos {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(62.9896, FieldConstants.FIELD_Y_MAX - FieldConstants.ROBOT_CENTER_Y, 90);
    private final Pose readyShoot = poseFactory.of(46.75, 118.75, 110);
    private final Pose readyShootControl1 = poseFactory.of(64.4074, 129.049, 0);
    private final Pose harvest = poseFactory.of(46.75, 129.5, 90);
    private final Pose shootToo = poseFactory.of(46.75, 118.75, 110);
    private final Pose shootTooSegment1Heading = poseFactory.of(46.75, 118.75, 90);
    private final Pose shootTooSegment2Start = poseFactory.of(46.75, 118.75, 90);
    private final Pose shootTooSegment2End = poseFactory.of(46.75, 118.75, 110);
    private final Pose parkIt = poseFactory.of(13.75, 118.75, 180);

    public Path readyShoot() {
        return Paths.curve(start, readyShootControl1, readyShoot).linear(start, readyShoot);
    }

    public Path harvest() {
        return Paths.line(readyShoot, harvest).constant(harvest);
    }

    public Path shootToo() {
        return Paths.line(harvest, shootToo).heading(Interpolator.piecewise().until(0.5091, Interpolator.constant(shootTooSegment1Heading)).until(1, Interpolator.linear(shootTooSegment2Start, shootTooSegment2End)));
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
        double distance = 57;
        return new SequentialCommandGroup(
                drive.followPathCmd(readyShoot()),
                waitForTagsCmd(12000),
                new IfCommand(this::getTagSeen,
                        // Tags Seen == TRUE
                        new SequentialCommandGroup(
                                shootSequenceCmd(distance),
                                startHarvesterCmd(),
                                drive.followPathCmd(harvest()),
                                waitCmd(2000),
                                drive.followPathCmd(shootToo()),
                                stopHarvesterCmd(),
                                shootSequenceCmd(distance)
                        )
                ),

                drive.followPathCmd(parkIt(), true)
        );

    }
}
