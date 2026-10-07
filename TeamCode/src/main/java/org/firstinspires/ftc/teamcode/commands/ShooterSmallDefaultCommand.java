//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.subsystems.ShooterSmallSubsystem;
//import org.stealthrobotics.library.opmodes.StealthOpMode;
//
//import java.util.function.DoubleSupplier;
//
//public class ShooterSmallDefaultCommand extends CommandBase {
//    private final ShooterSmallSubsystem shooterSmall;
//    private final Telemetry telemetry = StealthOpMode.telemetry;
//
//    private final DoubleSupplier leftTrigger;
//    private final DoubleSupplier rightTrigger;
//    private boolean manualControl = false;
//
//    private static final double axisDeadZone = 0.05;
//
//    public ShooterSmallDefaultCommand(
//            ShooterSmallSubsystem shooterSmall,
//            DoubleSupplier leftTrigger,
//            DoubleSupplier rightTrigger){
//
//        this.shooterSmall = shooterSmall;
//        this.leftTrigger = leftTrigger;
//        this.rightTrigger = rightTrigger;
//
//    }
//
//    @Override
//    public void execute() {
//        double leftPower = leftTrigger.getAsDouble();
//        double rightPower = rightTrigger.getAsDouble();
//        double power = rightPower - leftPower;
//
//        if (power > axisDeadZone || power < -axisDeadZone) {
//            manualControl = true;
//            shooterSmall.setPower(power);
//            telemetry.addData("Manual Shot", power);
//        }
//        else if (manualControl) {
//            shooterSmall.setPower(0);
//            manualControl = false;
//        }
//    }
//
//}
