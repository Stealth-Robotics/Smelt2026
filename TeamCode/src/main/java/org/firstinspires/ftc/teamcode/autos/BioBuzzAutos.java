package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.tools.AutoToTeleOpStorage;
import org.stealthrobotics.library.opmodes.StealthOpMode;
import com.pedropathing.paths.Path;

public class BioBuzzAutos extends StealthOpMode {

    public RobotSystem robot;
    public Follower follower;

    public DriveCommand driveCmd;
    public ShootCommand shooterCmd;
    @Override
    public void initialize() {
        robot = new RobotSystem(hardwareMap);
        follower = robot.getDriveSubsystem().getFollower();
        driveCmd = robot.getDriveCommand();
        shooterCmd = robot.getShootCommand();
        AutoToTeleOpStorage.reset();
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
        return driveCmd.aimAtTargetCommand(10000)
                .andThen(shooterCmd.shootWhenReadyCommand(100));
    }

}
