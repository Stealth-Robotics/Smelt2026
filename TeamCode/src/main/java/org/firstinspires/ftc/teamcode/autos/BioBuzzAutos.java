package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleShooterSubsystem;
import org.firstinspires.ftc.teamcode.tools.AutoToTeleOpStorage;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class BioBuzzAutos extends StealthOpMode {

    public RobotSystem robot;
    public Follower follower;

    public DriveSubsystem drive;

    public IntakeSubsystem intake;

    public SingleShooterSubsystem shooter;

    public CameraSubsystem camera;

    @Override
    public void initialize() {
        robot = new RobotSystem(hardwareMap, null, null);
        follower = robot.getDrive().getFollower();
        shooter = robot.getShooter();
        drive = robot.getDrive();
        intake = robot.getIntake();
        camera = robot.getCamera();

        AutoToTeleOpStorage.reset();
    }

    @Override
    public void whileWaitingToStart() {
        robot.updateCameraPipeline();
        Pose pose = drive.getPose();

        telemetry.addData("Alliance", Alliance.get());
        telemetry.addData("Camera Pipeline", robot.getCamera().getCurrentPipeline());
        telemetry.addData("Current Pose", "Tx: %.2f, Ty: %.2f, Z: %.2f", pose.x(), pose.y(),  Math.toDegrees(pose.heading()));
    }

    @Override
    public void opModeEnded() {
        AutoToTeleOpStorage.setPose(follower.pose());
    }

    @Override
    public void updateStorage() {
        AutoToTeleOpStorage.setPose(follower.pose());
    }

    public Command shootSequence(){
        return drive.aimAtTargetCommand(1000)
                .andThen(shooter.shootWhenReadyCommand(2000));
    }

    public Command waitFor(long ms) {
        return new WaitCommand(ms);
    }

    public Command waitForTags(long timeout) {
        return new WaitUntilCommand(camera::areTagsVisible).withTimeout(timeout);
    }

}
