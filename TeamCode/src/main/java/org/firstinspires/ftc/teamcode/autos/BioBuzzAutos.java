package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DmShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.tools.AutoToTeleOpStorage;
import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class BioBuzzAutos extends StealthOpMode {

    protected static final Pose FIELD_CENTER = new Pose(FieldConstants.FIELD_X_MAX / 2, FieldConstants.FIELD_Y_MAX / 2);

    public RobotSystem robot;
    public Follower follower;

    public DriveSubsystem drive;

    public IntakeSubsystem intake;

    public DmShooterSubsystem shooter;

    public CameraSubsystem camera;

    private boolean tagSeen = false;

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

    public Command shootSequenceCmd(){
        return drive.aimAtTargetCmd(500)
                .andThen(shooter.shootWhenReadyCmd(3000));
    }

    public Command shootSequenceCmd(double distance){
        return shootSequenceCmd(distance, 500);
    }

    public Command shootSequenceCmd(double distance, long aimTimeMs) {
        return drive.aimAtTargetCmd(aimTimeMs)
                .andThen(shooter.shootWhenReadyCmd(3000, distance));
    }

    public Command waitCmd(long ms) {
        return new WaitCommand(ms);
    }

    protected boolean getTagSeen() {
        return tagSeen;
    }

    public Command waitForTagsCmd(long timeout) {
        return new WaitUntilCommand(()-> {
            tagSeen = camera.areTagsVisible();
            return tagSeen;
        }).withTimeout(timeout);
    }

    public Command startHarvesterCmd(){
        return robot.getHarvesterSubsystem().harvDownCmd().andThen(
                intake.startAutoIntakeCommand(true)
        );
    }

    public Command stopHarvesterCmd(){
        return intake.stopAutoIntakeCommand().andThen(
                robot.getHarvesterSubsystem().harvUpCmd()
        );
    }

}
