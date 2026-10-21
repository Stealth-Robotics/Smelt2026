package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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
        Pose finalPose = robot.getDriveSubsystem().getFollower().pose();
        AutoToTeleOpStorage.setPose(finalPose);
    }

    public Command shootSequence(){
        return new SequentialCommandGroup(
               new InstantCommand(()-> driveCmd.setAutoAim(true)),
               new WaitUntilCommand(()-> driveCmd.isAimed()).withTimeout(500),
                shooterCmd.StartShootCommand(),
                new WaitCommand(1000),
                new InstantCommand(()-> driveCmd.setAutoAim(false)),
                shooterCmd.StopShootCommand());
    }

}
