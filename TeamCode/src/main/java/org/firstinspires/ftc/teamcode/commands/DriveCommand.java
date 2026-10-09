package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotate;
    private double power = 1.0;
    private boolean robotCentric = false;

    private static double slowpower = 0.3;
    private static double maxpower = 1.0;


    private boolean slomo = false;
    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate){
        this.driveSubsystem = driveSubsystem;
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
    }

    @Override
    public void execute(){
        Follower follower = driveSubsystem.getFollower();
        double forwardPow = forward.getAsDouble()*power;
        double strafePow = strafe.getAsDouble()*power;
        double rotatePow = rotate.getAsDouble()*power;
        DrivePowers powers = robotCentric?
                new DrivePowers(forwardPow, strafePow, rotatePow):
                ManualDrive.fieldCentric(forwardPow, strafePow, rotatePow, follower.pose().heading());
        follower.manual(powers);

    }

    public void toggleSlomo(){
        slomo = !slomo;
        power = slomo? slowpower : maxpower;

    }

    public void toggleRobotCentric(){
        robotCentric = !robotCentric;
    }

}
