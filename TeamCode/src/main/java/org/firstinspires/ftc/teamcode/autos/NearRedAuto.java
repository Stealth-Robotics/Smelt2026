package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "testauto")
public class NearRedAuto extends BioBuzzAutos {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.1326, 8.2169, 90);
    private final Pose goShoot = poseFactory.of(55, 32, 90);
    private final Pose goToIntakeStart = poseFactory.of(55, 32, 90);
    private final Pose goToIntake = poseFactory.of(20, 8, 180);
    private final Pose intakeStart = poseFactory.of(20, 8, 180);
    private final Pose intaking = poseFactory.of(8, 8, 180);
    private final Pose goShoot2Start = poseFactory.of(8, 8, 180);
    private final Pose goShoot2 = poseFactory.of(55, 32, 90);

    public Path goShoot() {
        return Paths.line(start, goShoot).linear(start, goShoot);
    }

    public Path goToIntake() {
        return Paths.line(goToIntakeStart, goToIntake).linear(goToIntakeStart, goToIntake);
    }

    public Path intake() {
        return Paths.line(intakeStart, intaking).linear(intakeStart, intaking);
    }

    public Path goShoot2() {
        return Paths.line(goShoot2Start, goShoot2).linear(goShoot2Start, goShoot2);
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
