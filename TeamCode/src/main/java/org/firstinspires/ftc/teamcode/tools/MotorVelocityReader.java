package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;
import java.util.Queue;

/**
 * For calculating the TRUE RPM of a motor with filtering for smoother readings.
 */
public class MotorVelocityReader {
    private final DcMotorEx motor;
    private final double ticksPerRev;
    private final ElapsedTime timer;
    private int lastPosition;
    private double lastTime;
    private double lastRpm = 0;
    private final MovingAverageFilter rpmFilter;

    /**
     * @param motor The motor to measure.
     * @param ticksPerRev The number of encoder ticks per revolution for this motor.
     * @param timer A shared ElapsedTime timer.
     * @param filterSize The number of samples to average for the moving average filter.
     */
    public MotorVelocityReader(DcMotorEx motor, double ticksPerRev, ElapsedTime timer, int filterSize) {
        this.motor = motor;
        this.ticksPerRev = ticksPerRev;
        this.timer = timer;
        this.rpmFilter = new MovingAverageFilter(filterSize);
        reset();
    }

    /**
     * @param motor The motor to measure.
     * @param ticksPerRev The number of encoder ticks per revolution for this motor.
     * @param timer A shared ElapsedTime timer. Defaults filter size to 5.
     */
    public MotorVelocityReader(DcMotorEx motor, double ticksPerRev, ElapsedTime timer) {
        this(motor, ticksPerRev, timer, 5); // Default filter size
    }

    /**
     * Constructor that creates its own ElapsedTime timer.
     * @param motor The motor to measure.
     * @param ticksPerRev The number of encoder ticks per revolution for this motor.
     */
    public MotorVelocityReader(DcMotorEx motor, double ticksPerRev) {
        this(motor, ticksPerRev, new ElapsedTime(), 5);
    }

    /**
     * Resets the baseline for velocity calculation. This should be called once
     * when you are ready to start measuring, for example, at the beginning of runOpMode.
     */
    public void reset() {
        lastPosition = motor.getCurrentPosition();
        lastTime = timer.seconds();
        lastRpm = 0;
        rpmFilter.clear();
    }

    /**
     * Calculates the motor's current raw RPM based on encoder ticks and time elapsed.
     * This should be called periodically in a loop.
     * Consider using getFilteredRpm() for a more stable reading.
     * @return The current, measured, unfiltered RPM of the motor.
     */
    public double getRpm() {
        double currentTime = timer.seconds();
        int currentPosition = motor.getCurrentPosition();
        double deltaTime = currentTime - lastTime;

        // If not enough time has passed, return the previously calculated RPM.
        // This prevents division by zero and noisy readings from small time deltas.
        if (deltaTime < 0.025) { // Increased threshold for more reliable readings
            return lastRpm;
        }

        int deltaTicks = currentPosition - lastPosition;
        double ticksPerSecond = deltaTicks / deltaTime;
        double revolutionsPerSecond = ticksPerSecond / ticksPerRev;
        double currentRpm = revolutionsPerSecond * 60;

        // Update state for the next calculation
        lastTime = currentTime;
        lastPosition = currentPosition;
        lastRpm = currentRpm; // Store the newly calculated raw RPM

        rpmFilter.add(currentRpm);

        return currentRpm;
    }

    /**
     * Calculates and returns the filtered RPM of the motor.
     * This method is preferred over getRpm() for smoother, more stable velocity readings,
     * which are ideal for control loops.
     * @return The filtered (averaged) RPM of the motor.
     */
    public double getFilteredRpm() {
        // Ensure getRpm() is called to update the filter with the latest value
        getRpm();
        return rpmFilter.getAverage();
    }

    /**
     * A simple moving average filter to smooth out noisy data.
     */
    private static class MovingAverageFilter {
        private final Queue<Double> window = new LinkedList<>();
        private final int size;
        private double sum = 0.0;

        public MovingAverageFilter(int size) {
            this.size = Math.max(1, size); // Ensure size is at least 1
        }

        public void add(double value) {
            sum += value;
            window.add(value);
            if (window.size() > size) {
                sum -= window.poll();
            }
        }

        public double getAverage() {
            if (window.isEmpty()) {
                return 0.0;
            }
            return sum / window.size();
        }

        public void clear() {
            window.clear();
            sum = 0.0;
        }
    }
}