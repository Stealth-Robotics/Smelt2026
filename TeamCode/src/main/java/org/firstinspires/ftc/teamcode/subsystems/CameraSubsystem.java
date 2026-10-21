package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.tools.Pipeline.*;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.firstinspires.ftc.teamcode.tools.Pipeline;

import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.ArrayList;
import java.util.List;

public class CameraSubsystem extends StealthSubsystem {

    private final Telemetry telemetry = StealthOpMode.telemetry;
    public static final String NAME = "limelight";
    private  final Limelight3A limelight;
    /** The length of time to keep LLResults in the queue before removing them, in milliseconds. */
    private static final long QUEUE_DEFAULT_TIMEOUT_MS = 500;

    /** Rotational difference in degrees between the Limelight's yaw and the robot's IMU yaw. */
    private static final double LL_ANGLE_DELTA = 90;

    /** How many degrees back the Limelight is rotated from perfectly vertical. */
    private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 40;

    /** Distance from the center of the Limelight lens to the floor, in inches. */
    private static final double LIMELIGHT_FLOOR_HEIGHT_INCHES = 11.5;

    /** The height of the center of an AprilTag from the floor, in inches. */
    private static final double APRIL_TAG_CENTER_HEIGHT_INCHES = 49.65551;

    /** Physical Y offset of the Limelight lens relative to the shooter/robot center (in inches).
     *  Positive value indicates the camera is mounted to the LEFT of the shooter. */
    public static final double LIMELIGHT_Y_OFFSET_INCHES = 2.2;

    /** The last valid result received from the Limelight. */
    private LLResult lastResult;


    public CameraSubsystem (HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, NAME);
        // setPollRate must be called before start
        limelight.setPollRateHz(250);
        limelight.start();
    }

    @Nullable
    public LLResult getLastResult() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            this.lastResult = result;
        }

        return this.lastResult != null && lastResult.getStaleness() < 40 ? lastResult : null;
    }

    @Override
    public void periodic(){
        LLResult result = getLastResult();
        telemetry.addData("Pipeline:", getCurrentPipeline());
        if (result != null && result.isValid()) {
////            telemetry.addData("Latency ms", result.getStaleness());
            telemetry.addData("Tags:", getTagIds(result));
////            telemetry.addData("Tx:", result.getTx());
//            telemetry.addData("Adjusted Tx", getAdjustedTx(result));
//            telemetry.addData("Ta Distance", calcGoalDistanceByTa(result));
////            telemetry.addData("Ty:", result.getTy());
////            telemetry.addData("Area:", result.getTa());
////            telemetry.addData("TY Distance", calcGoalDistanceByTy(result.getTy()));
////            telemetry.addData("TagIds", getTagIds(result));
        } else {
            telemetry.addData("Tags:", "None");
        }
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public void updateRobotOrientation(double pedroHeading){
        double yaw = getCorrectedYaw(Math.toDegrees(pedroHeading));
        limelight.updateRobotOrientation(yaw);
    }

    /**
     * Gets the currently active pipeline on the Limelight.
     *
     * @return The current {@link Pipeline}.
     */
    public Pipeline getCurrentPipeline() {
        return Pipeline.values()[limelight.getStatus().getPipelineIndex()];
    }

    public void toggleAllTagsPipeline() {
        Pipeline cur = getCurrentPipeline();
        switch (cur) {
            case RED_ALL:
                setPipeline(BLUE_ALL);
                break;
            case BLUE_ALL:
                setPipeline(RED_FAR);
                break;
            case RED_FAR:
                setPipeline(RED_NEAR);
                break;
                case RED_NEAR:
                    setPipeline(BLUE_NEAR);
                break;
            case BLUE_NEAR:
                setPipeline(BLUE_FAR);
                break;
            default:
                setPipeline(RED_ALL);
                break;
        }
    }

    /**
     * Switches the Limelight's active pipeline.
     *
     * @param pipeline The {@link Pipeline} to switch to.
     */
    public void setPipeline(@NonNull Pipeline pipeline) {
        telemetry.addData("Pipeline Switch Request", pipeline);
        if (pipeline == getCurrentPipeline()) {
            return;
        }

        limelight.pipelineSwitch(pipeline.id);
        this.lastResult = null;
    }

    /**
     * Calculates distance to a goal by using the cameras Ta value using 4th order polynomial
     * @param result The LLResult from Limelight
     * @return distance in inches
     */
    public double calcGoalDistanceByTa(LLResult result) {
        double ta = getAverageTa(result);

        double distance = 141.857 - 42948.86 * ta + 7811325.0  * Math.pow(ta, 2) - 746920500.0  * Math.pow(ta, 3) + 29360910000.0  * Math.pow(ta, 4);
        return Math.round(distance * 100) / 100.0; // round to 2 decimal places)
    }

    /**
     * Calculates distance to a goal by using the cameras Ty value
     * @param ty The Ty value from the camera
     *
     * @return The distance to the goal in inches
     */
    public static double calcGoalDistanceByTy(double ty) {
        // This is the total angle from the camera to the target, relative to a horizontal line from the camera.
        double totalAngleDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;

        // Convert the total angle to radians for use in Math.tan()
        double totalAngleRadians = Math.toRadians(totalAngleDegrees);

        // Calculate the height difference between the AprilTag and the Limelight
        double heightDifference = APRIL_TAG_CENTER_HEIGHT_INCHES - LIMELIGHT_FLOOR_HEIGHT_INCHES;

        // Using trigonometry (tangent) to find the horizontal distance.
        // distance = opposite / tan(angle)

        return heightDifference / Math.tan(totalAngleRadians);
    }

    public double getAverageTa(@Nullable LLResult result) {
        if (result == null || !result.isValid()) {
            return 0.0;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return 0.0;
        }
        List<Double> areas = new ArrayList<>();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            areas.add(fr.getTargetArea());
        }
        if (areas.isEmpty()) {
            return 0.0;
        }

        return calculateAverage(areas);
    }

    private List<Integer> getTagIds(LLResult result ) {
        List<Integer> tags = new ArrayList<>();
        if (result == null || !result.isValid()) {
            return tags;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        for (int i = 0; i < fiducials.size(); i++) {
            tags.add(fiducials.get(i).getFiducialId());
        }

        return tags;
    }

    /**
     * Lookup table mapping tag visibility bitmask (0 to 15) to percentage offset of visible tag span.
     * Bit 0 = leftmost tag (rel index 0), Bit 1 = middle-left (rel 1), Bit 2 = middle-right (rel 2), Bit 3 = rightmost (rel 3).
     * Positive shifts right, negative shifts left.
     */
    private static final double[] PERCENT_OFFSET_LOOKUP = {
        0.0,            // 0: None
        0.0,            // 1: {0} (single tag)
        0.0,            // 2: {1} (single tag)
        1.0,            // 3: {0,1} -> shift right +100% of visible span
        0.0,            // 4: {2} (single tag)
        0.25,           // 5: {0,2} -> shift right +25% of visible span
        0.0,            // 6: {1,2} -> balanced middle tags (0%)
        0.25,           // 7: {0,1,2} -> shift right +25% of visible span
        0.0,            // 8: {3} (single tag)
        0.0,            // 9: {0,3} -> balanced outer tags (0%)
        -0.25,          // 10: {1,3} -> shift left -25% of visible span
        1.0 / 18.0,     // 11: {0,1,3} -> shift right +5.56% of visible span
        -1.0,           // 12: {2,3} -> shift left -100% of visible span
        -1.0 / 18.0,    // 13: {0,2,3} -> shift left -5.56% of visible span
        -0.25,          // 14: {1,2,3} -> shift left -25% of visible span
        0.0             // 15: {0,1,2,3} -> balanced all 4 tags (0%)
    };

    /**
     * Lookup table mapping tag visibility bitmask (0 to 15) to offset in tag-spacing units.
     * Used for single-tag fallbacks or spacing-based calculations.
     */
    private static final double[] SPACING_OFFSET_LOOKUP = {
        0.0,            // 0: None
        1.5,            // 1: {0} -> target center is 1.5 spacings right
        0.5,            // 2: {1} -> target center is 0.5 spacings right
        1.0,            // 3: {0,1}
        -0.5,           // 4: {2} -> target center is 0.5 spacings left
        0.5,            // 5: {0,2}
        0.0,            // 6: {1,2}
        0.5,            // 7: {0,1,2}
        -1.5,           // 8: {3} -> target center is 1.5 spacings left
        0.0,            // 9: {0,3}
        -0.5,           // 10: {1,3}
        1.0 / 6.0,      // 11: {0,1,3}
        -1.0,           // 12: {2,3}
        -1.0 / 6.0,     // 13: {0,2,3}
        -0.5,           // 14: {1,2,3}
        0.0             // 15: {0,1,2,3}
    };

    /**
     * Returns the relative index (0 to 3, from left to right facing the hive)
     * of an AprilTag within its hive group.
     *
     * Subtracts the hive's base AprilTag ID:
     * - RED_FAR:   IDs 30..33 -> base 30
     * - RED_NEAR:  IDs 34..37 -> base 34
     * - BLUE_NEAR: IDs 38..41 -> base 38
     * - BLUE_FAR:  IDs 42..45 -> base 42
     *
     * @param tagId The AprilTag ID.
     * @return Relative index 0..3, or -1 if not a hive tag.
     */
    public static int getRelativeTagIndex(int tagId) {
        if (tagId >= 30 && tagId <= 33) return tagId - 30; // RED_FAR
        if (tagId >= 34 && tagId <= 37) return tagId - 34; // RED_NEAR
        if (tagId >= 38 && tagId <= 41) return tagId - 38; // BLUE_NEAR
        if (tagId >= 42 && tagId <= 45) return tagId - 42; // BLUE_FAR
        return -1;
    }

    /**
     * Helper method that calculates the tag visibility bitmask (0..15) and
     * locates the leftmost/rightmost detected tags for a given LLResult.
     *
     * @param result The LLResult.
     * @return An integer array containing [bitmask, minIdx, maxIdx], or [0, -1, -1] if invalid.
     */
    @NonNull
    private HiveDetectionData processHiveDetections(@Nullable LLResult result) {
        if (result == null || !result.isValid()) {
            return new HiveDetectionData(0, null, null, -1, -1);
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return new HiveDetectionData(0, null, null, -1, -1);
        }

        int bitmask = 0;
        LLResultTypes.FiducialResult minTag = null;
        LLResultTypes.FiducialResult maxTag = null;
        int minIdx = 4;
        int maxIdx = -1;

        for (LLResultTypes.FiducialResult fr : fiducials) {
            int relIdx = getRelativeTagIndex(fr.getFiducialId());
            if (relIdx >= 0 && relIdx <= 3) {
                bitmask |= (1 << relIdx);
                if (relIdx < minIdx) {
                    minIdx = relIdx;
                    minTag = fr;
                }
                if (relIdx > maxIdx) {
                    maxIdx = relIdx;
                    maxTag = fr;
                }
            }
        }

        if (maxIdx == -1) {
            return new HiveDetectionData(0, null, null, -1, -1);
        }

        return new HiveDetectionData(bitmask, minTag, maxTag, minIdx, maxIdx);
    }

    private static class HiveDetectionData {
        final int bitmask;
        final LLResultTypes.FiducialResult minTag;
        final LLResultTypes.FiducialResult maxTag;
        final int minIdx;
        final int maxIdx;

        HiveDetectionData(int bitmask, LLResultTypes.FiducialResult minTag, LLResultTypes.FiducialResult maxTag, int minIdx, int maxIdx) {
            this.bitmask = bitmask;
            this.minTag = minTag;
            this.maxTag = maxTag;
            this.minIdx = minIdx;
            this.maxIdx = maxIdx;
        }
    }

    /**
     * Calculates the offset as a percentage/fraction of the visible tag span using the lookup table.
     * For example, if left two tags are seen (bitmask 3), returns +1.0 (+100% shift right).
     *
     * @param result The LLResult from Limelight.
     * @return The percentage offset from the lookup table (-1.0 to +1.0).
     */
    public double getAimingOffsetPercentage(@Nullable LLResult result) {
        HiveDetectionData data = processHiveDetections(result);
        return PERCENT_OFFSET_LOOKUP[data.bitmask];
    }

    /**
     * Calculates the aiming offset in degrees using the lookup table.
     *
     * @param result The LLResult from Limelight.
     * @param fallbackDegreesPerSpacing Default degrees per tag spacing to use if only 1 tag is detected.
     * @return The offset in degrees to add to result.getTx().
     */
    public double getAimingOffsetDegrees(@Nullable LLResult result, double fallbackDegreesPerSpacing) {
        HiveDetectionData data = processHiveDetections(result);

        if (data.bitmask == 0) {
            return 0.0;
        }

        // If 2 or more tags visible, compute degrees per spacing from the detected outer tags
        if (data.maxIdx > data.minIdx && data.minTag != null && data.maxTag != null) {
            double angleSpan = data.maxTag.getTargetXDegrees() - data.minTag.getTargetXDegrees();
            return PERCENT_OFFSET_LOOKUP[data.bitmask] * angleSpan;
        } else {
            // Single tag visible - use spacing lookup table and fallback conversion factor
            return SPACING_OFFSET_LOOKUP[data.bitmask] * fallbackDegreesPerSpacing;
        }
    }

    /**
     * Calculates the aiming offset in degrees using the lookup table.
     *
     * @param result The LLResult from Limelight.
     * @return The offset in degrees to add to result.getTx().
     */
    public double getAimingOffsetDegrees(@Nullable LLResult result) {
        return getAimingOffsetDegrees(result, 0.0);
    }

    /**
     * Calculates the angular correction in degrees needed to compensate for the Limelight's
     * physical offset relative to the shooter centerline (4.5 inches to the left).
     *
     * Uses distance calculated from ty (calcGoalDistanceByTy).
     *
     * @param result The LLResult from Limelight.
     * @return The angular offset in degrees to add to tx.
     */
    public double getPhysicalCameraOffsetDegrees(@Nullable LLResult result) {
        if (result == null || !result.isValid()) {
            return 0.0;
        }

        double distanceInches = calcGoalDistanceByTa(result);
        if (distanceInches <= 0.0 || Double.isNaN(distanceInches)) {
            return 0.0;
        }

        // Camera is to the left of shooter (+4.5 inches).
        // Target appears to the right of camera crosshair when shooter is aligned.
        // Therefore, we subtract atan(4.5 / distance) from camera tx.
        double offsetAngleRadians = Math.atan2(LIMELIGHT_Y_OFFSET_INCHES, distanceInches);
        return -Math.toDegrees(offsetAngleRadians);
    }

    /**
     * Gets the adjusted tx angle in degrees aimed directly from the shooter
     * to the center of the hive target, compensating for both:
     * 1) Imbalanced AprilTag grouping (hive center offset)
     * 2) Physical camera offset (4.5 inches left of shooter)
     *
     * @param result The LLResult from Limelight.
     * @return The corrected tx angle for the shooter.
     */
    public double getAdjustedTx(@Nullable LLResult result) {
        if (result == null || !result.isValid()) {
            return 0.0;
        }

        double hiveGroupOffset = getAimingOffsetDegrees(result);
        double physicalOffset = getPhysicalCameraOffsetDegrees(result);

        return result.getTx() + hiveGroupOffset  + physicalOffset;
    }


    @NonNull
    public static Pose limelightToPedroPose(@NonNull Pose3D botPose) {

        double llYaw = botPose.getOrientation().getYaw(AngleUnit.DEGREES);
        // Rotoate the limlight by LL_ANGLE_DELTA to get the robot's heading
        double imuAngle = (llYaw < 0) ? (360 + llYaw - LL_ANGLE_DELTA) : (llYaw - LL_ANGLE_DELTA);

        // Ensure heading is within the 0-360 degree range
        imuAngle %= 360;

        // Convert position from Limelight's coordinate system (meters, field center)
        // to the path follower's system (inches, red alliance corner).
        Position position = botPose.getPosition().toUnit(DistanceUnit.INCH);

        // Note: X and Y are swapped and adjusted for the different origins.
        return new Pose(
                (FieldConstants.FIELD_Y_MAX / 2.0) - (-position.y),
                -position.x + (FieldConstants.FIELD_X_MAX / 2.0),
                Math.toRadians(imuAngle));
    }

    /**
     * Extracts the robot's yaw angle corrected for Limelight mounting offset.
     * This is the yaw value suitable for robot localizer updates.
     *
     * @param pedroYaw The base yaw (in degrees) from the Pedro system/IMU.
     * @return The corrected yaw in degrees (0-360).
     */
    public double getCorrectedYaw(double pedroYaw) {
        // Rotate the limelight by LL_ANGLE_DELTA to get the robot's heading
        double correctedYaw = (pedroYaw < 0) ? (360 + pedroYaw - LL_ANGLE_DELTA) : (pedroYaw - LL_ANGLE_DELTA);
        // Ensure heading is within the 0-360 degree range
        return correctedYaw % 360;
    }

    /**
     * Calculates the average of a list of values
     * @param values list of values
     * @return average
     */
    private static double calculateAverage(List<Double> values) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (Double value : values) {
            sum += value;
        }
        return sum / values.size();
    }

    /**
     * Calculates the standard deviation of a list of values
     * @param values list of values
     * @param average average of the values
     * @return standard deviation
     */
    private static double calculateStandardDeviation(List<Double> values, double average) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double sumOfSquaredDeviations = 0.0;
        for (Double value : values) {
            sumOfSquaredDeviations += Math.pow(value - average, 2);
        }
        return Math.sqrt(sumOfSquaredDeviations / (values.size() - 1));
    }

}
