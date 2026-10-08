package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveTrainCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotate;
    private Double power = 1.0;
    public DriveTrainCommand(DriveSubsystem driveSubsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate){
        this.driveSubsystem = driveSubsystem;
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
    }

    @Override
    public void execute(){
        Follower follower = driveSubsystem.getFollower();


    }

}
