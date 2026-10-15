package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tools.MotorVelocityReader;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class ShooterSmallSubsystem extends StealthSubsystem {

    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private static final double MOTOR_TICKS_REV = 28;

    private final Telemetry telemetry = StealthOpMode.telemetry;

    private final MotorVelocityReader motorVelocityReader1;
    private final MotorVelocityReader motorVelocityReader2;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private final ElapsedTime shootTimer = new ElapsedTime();

    private double VELOCITY_TOLERANCE_HIGH = 100;
    public static final double MAX_RPM = 4500;
    public static final double MIN_RPM = -4000;
    private static final double MIN_SHOOT_TIME_MS = 500;
    private static final double MAX_SHOOT_TIME_MS = 5000;
    private double setRpms = 0;
    private double currentRpm = 0;


    public static final PIDFController ShooterController = new PIDFController(0.009, 0.0001, 0.0001, 0.00052);
    public ShooterSmallSubsystem(HardwareMap hardwareMap) {

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");

        motorVelocityReader1 = new MotorVelocityReader(shooterMotor1, MOTOR_TICKS_REV);
        motorVelocityReader2 = new MotorVelocityReader(shooterMotor2, MOTOR_TICKS_REV);

    }
    public boolean isShootReady(double targetRpm) {
        double difference = Math.abs(currentRpm - targetRpm);
        double curMs = shootTimer.milliseconds();
        if (curMs < MIN_SHOOT_TIME_MS) {
            return false;
        }
        return (difference <= VELOCITY_TOLERANCE_HIGH||curMs > MAX_SHOOT_TIME_MS);
    }
    public void update() {
        updateAverageRpm();
    }

    public void setRpm(double rpm) {
        // Clamp the RPM to the allowable min/max range to prevent motor damage or unexpected behavior.
        if (rpm > MAX_RPM) {
            rpm = MAX_RPM;
        } else if (rpm < MIN_RPM) {
            rpm = MIN_RPM;
        }
        setRpms = rpm;
        // Convert desired RPM to encoder ticks per second, which is the unit required by DcMotorEx.setVelocity().
        double ticksPerSecond = rpm * MOTOR_TICKS_REV / 60;
        ShooterController.reset();
        ShooterController.setSetPoint(ticksPerSecond);
        resetVelocityReaders();
    }

    public double getCurrentRpm() {
        return ((((Math.abs(shooterMotor1.getVelocity()) + Math.abs(shooterMotor2.getVelocity())) / 2.0) / MOTOR_TICKS_REV) * 60);
    }

    public double getAvgTicks(){
        return (shooterMotor1.getVelocity()+shooterMotor2.getVelocity())/2;
    }

    public double getMotorRpms() {
        return (((this.shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2.0) / MOTOR_TICKS_REV) * 60;
    }

    private void resetVelocityReaders(){
        motorVelocityReader1.reset();
        motorVelocityReader2.reset();
        shootTimer.reset();
    }

    private void updateAverageRpm() {
        // Use the absolute value to prevent issues with one motor reporting a negative value momentarily
        double motorRpm1 = motorVelocityReader1.getFilteredRpm();
        double motorRpm2 = motorVelocityReader2.getFilteredRpm();
        this.currentRpm = (Math.abs(motorRpm1) + Math.abs(motorRpm2)) / 2.0;
    }
    @Override
    public void periodic() {
        update();
        double calculatedPower= ShooterController.calculate(-getAvgTicks());
        dashboardTelemetry.addData("AVG TICKS:", -getAvgTicks());
        dashboardTelemetry.addData("calculatedPower", calculatedPower);
        calculatedPower = MathUtils.clamp(calculatedPower, -1, 1);
        dashboardTelemetry.addData("setRpms", setRpms);
        dashboardTelemetry.addData(
                "CurrentRPM", getCurrentRpm());
        dashboardTelemetry.update();
        dashboardTelemetry.addData("setRpms", setRpms);
        dashboardTelemetry.addData("CurrentRPM", getCurrentRpm());
        dashboardTelemetry.addData("CurrentRpmTest", currentRpm);
        dashboardTelemetry.addData("isShootReady", isShootReady(3700));
        dashboardTelemetry.addData("AverageRpm", getCurrentRpm());
        dashboardTelemetry.addData("Motor1Rpm", Math.abs(shooterMotor1.getVelocity() / MOTOR_TICKS_REV * 60));
        dashboardTelemetry.addData("Motor2Rpm", Math.abs(shooterMotor2.getVelocity() / MOTOR_TICKS_REV * 60));
        shooterMotor1.setPower(calculatedPower);
        shooterMotor2.setPower(calculatedPower);
        dashboardTelemetry.update();
    }
}
