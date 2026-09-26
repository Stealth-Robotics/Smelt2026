package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSmallSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterSmallDefaultCommand extends CommandBase {
    private final ShooterSmallSubsystem shooterSmall;
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private final DoubleSupplier leftTrigger;
    private final DoubleSupplier rightTrigger;
    private boolean manualControl = false;

    private static final double axisDeadZone = 0.05;

    public ShooterSmallDefaultCommand(
            ShooterSmallSubsystem shooterSmall,
            DoubleSupplier leftTrigger,
            DoubleSupplier rightTrigger){

        this.shooterSmall = shooterSmall;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
    }

    @Override
    public void execute() {
        double leftPower = leftTrigger.getAsDouble();
        double rightPower = rightTrigger.getAsDouble();
        double power = rightPower - leftPower;

        if (power > axisDeadZone || power < -axisDeadZone) {
            manualControl = true;
            shooterSmall.setPower(power);
            telemetryM.addData("Manual Shot", power);
        }
        else if (manualControl) {
            shooterSmall.setPower(0);
            manualControl = false;
        }
    }

}
