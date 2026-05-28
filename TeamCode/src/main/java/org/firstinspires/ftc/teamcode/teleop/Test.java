package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@TeleOp(name = "Test - AprilTag Shoot Angle", group = "TeleOp2")
public class Test extends OpMode {
    public static int TARGET_TAG_ID = 20;
    public static int LIMELIGHT_PIPELINE = 0;
    public static int LIMELIGHT_POLL_RATE_HZ = 100;
    public static double MAX_RESULT_STALENESS_MS = 250.0;

    // Unit: meter. Tune these before trusting the predicted distance.
    public static double CAMERA_HEIGHT_M = 0.30;
    public static double APRILTAG_CENTER_HEIGHT_M = 0.90;
    public static double CAMERA_PITCH_DEG = 0.0;
    public static double DISTANCE_SCALE = 2.45 / 1.59;
    public static double DISTANCE_OFFSET_M = 0.0;
    public static double REFERENCE_DISTANCE_M = 2.45;
    public static double REFERENCE_SERVO_POS = 0.500;

    public static double SERVO_START_POS = 0.50;
    public static double SERVO_MIN_POS = 0.20;
    public static double SERVO_MAX_POS = 0.85;
    public static double SERVO_FINE_STEP = 0.001;
    public static double SERVO_COARSE_STEP = 0.010;
    public static double SERVO_MANUAL_RATE_PER_SEC = 0.25;

    public static double DISTANCE_FILTER_ALPHA = 0.45;

    private Servo shootAngle;
    private Limelight3A limelight;
    private int activePipeline = -1;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double shootAnglePos = SERVO_START_POS;
    private boolean resultValid = false;
    private boolean tagVisible = false;
    private int seenTagId = -1;
    private double tagTxDeg = 0.0;
    private double tagTyDeg = 0.0;
    private double tagArea = 0.0;
    private int botposeTagCount = 0;
    private double botposeAvgDistM = 0.0;
    private double rawHorizontalDistanceM = Double.NaN;
    private double rawDirectDistanceM = Double.NaN;
    private double predictedHorizontalDistanceM = Double.NaN;
    private double predictedDirectDistanceM = Double.NaN;
    private double filteredHorizontalDistanceM = Double.NaN;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        shootAngle = hardwareMap.get(Servo.class, "shoot angle");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(LIMELIGHT_POLL_RATE_HZ);
        applyLimelightPipeline();
        limelight.start();

        shootAnglePos = clamp(SERVO_START_POS, SERVO_MIN_POS, SERVO_MAX_POS);
        shootAngle.setPosition(shootAnglePos);

        loopTimer.reset();
        telemetry.addLine("Ready: AprilTag distance / shoot angle servo test");
        telemetry.update();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0.0 || dt > 0.25) {
            dt = 0.02;
        }

        applyLimelightPipeline();
        updateServoPosition(dt);
        updateAprilTagDistance();
        updateTelemetry();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    private void updateServoPosition(double dt) {
        double manual = -gamepad1.right_stick_y;
        if (Math.abs(manual) > 0.05) {
            shootAnglePos += manual * SERVO_MANUAL_RATE_PER_SEC * dt;
        }

        if (gamepad1.dpad_up && !lastDpadUp) {
            shootAnglePos += SERVO_COARSE_STEP;
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            shootAnglePos -= SERVO_COARSE_STEP;
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            shootAnglePos += SERVO_FINE_STEP;
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            shootAnglePos -= SERVO_FINE_STEP;
        }
        if (gamepad1.a && !lastA) {
            shootAnglePos = SERVO_START_POS;
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastA = gamepad1.a;

        shootAnglePos = clamp(shootAnglePos, SERVO_MIN_POS, SERVO_MAX_POS);
        shootAngle.setPosition(shootAnglePos);
    }

    private void updateAprilTagDistance() {
        resultValid = false;
        tagVisible = false;
        seenTagId = -1;
        tagTxDeg = 0.0;
        tagTyDeg = 0.0;
        tagArea = 0.0;
        botposeTagCount = 0;
        botposeAvgDistM = 0.0;
        rawHorizontalDistanceM = Double.NaN;
        rawDirectDistanceM = Double.NaN;
        predictedHorizontalDistanceM = Double.NaN;
        predictedDirectDistanceM = Double.NaN;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_RESULT_STALENESS_MS) {
            return;
        }

        resultValid = true;
        botposeTagCount = result.getBotposeTagCount();
        botposeAvgDistM = result.getBotposeAvgDist();

        LLResultTypes.FiducialResult tag = chooseTargetTag(result.getFiducialResults());
        if (tag == null) {
            return;
        }

        tagVisible = true;
        seenTagId = tag.getFiducialId();
        tagTxDeg = tag.getTargetXDegrees();
        tagTyDeg = tag.getTargetYDegrees();
        tagArea = tag.getTargetArea();

        rawHorizontalDistanceM = estimateHorizontalDistanceM(tagTyDeg);
        if (!Double.isNaN(rawHorizontalDistanceM)) {
            double heightDelta = APRILTAG_CENTER_HEIGHT_M - CAMERA_HEIGHT_M;
            rawDirectDistanceM = Math.hypot(rawHorizontalDistanceM, heightDelta);
            predictedHorizontalDistanceM = applyDistanceCalibration(rawHorizontalDistanceM);
            predictedDirectDistanceM = applyDistanceCalibration(rawDirectDistanceM);
            updateFilteredDistance(predictedHorizontalDistanceM);
        }
    }

    private LLResultTypes.FiducialResult chooseTargetTag(List<LLResultTypes.FiducialResult> fiducials) {
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }

        LLResultTypes.FiducialResult best = null;
        double bestAbsTx = Double.POSITIVE_INFINITY;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (TARGET_TAG_ID >= 0 && fiducial.getFiducialId() != TARGET_TAG_ID) {
                continue;
            }

            double absTx = Math.abs(fiducial.getTargetXDegrees());
            if (absTx < bestAbsTx) {
                bestAbsTx = absTx;
                best = fiducial;
            }
        }
        return best;
    }

    private double estimateHorizontalDistanceM(double targetYDeg) {
        double heightDelta = APRILTAG_CENTER_HEIGHT_M - CAMERA_HEIGHT_M;
        double angleRad = Math.toRadians(CAMERA_PITCH_DEG + targetYDeg);
        double tan = Math.tan(angleRad);
        if (Math.abs(tan) < 0.001) {
            return Double.NaN;
        }

        double distance = heightDelta / tan;
        return distance > 0.0 ? distance : Double.NaN;
    }

    private double applyDistanceCalibration(double rawDistanceM) {
        if (Double.isNaN(rawDistanceM)) {
            return Double.NaN;
        }
        return rawDistanceM * DISTANCE_SCALE + DISTANCE_OFFSET_M;
    }

    private void updateFilteredDistance(double distanceM) {
        double alpha = clamp(DISTANCE_FILTER_ALPHA, 0.0, 1.0);
        if (Double.isNaN(filteredHorizontalDistanceM)) {
            filteredHorizontalDistanceM = distanceM;
        } else {
            filteredHorizontalDistanceM = alpha * distanceM + (1.0 - alpha) * filteredHorizontalDistanceM;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Target Tag", TARGET_TAG_ID >= 0 ? TARGET_TAG_ID : "Any");
        telemetry.addData("Limelight", resultValid ? "Valid" : "No valid result");
        telemetry.addData("AprilTag", tagVisible ? "Visible" : "Not found");
        telemetry.addData("Seen Tag ID", seenTagId);
        telemetry.addData("Tag tx/ty", "%.2f / %.2f deg", tagTxDeg, tagTyDeg);
        telemetry.addData("Tag area", "%.2f", tagArea);
        telemetry.addData("Raw Distance", "%.2f m / %.1f in",
                rawHorizontalDistanceM, metersToInches(rawHorizontalDistanceM));
        telemetry.addData("Predicted Distance", "%.2f m / %.1f in",
                predictedHorizontalDistanceM, metersToInches(predictedHorizontalDistanceM));
        telemetry.addData("Filtered Distance", "%.2f m / %.1f in",
                filteredHorizontalDistanceM, metersToInches(filteredHorizontalDistanceM));
        telemetry.addData("Direct Tag Distance", "%.2f m / %.1f in",
                predictedDirectDistanceM, metersToInches(predictedDirectDistanceM));
        telemetry.addData("Raw Direct Distance", "%.2f m / %.1f in",
                rawDirectDistanceM, metersToInches(rawDirectDistanceM));
        telemetry.addData("Limelight AvgDist", "tags=%d dist=%.2f m",
                botposeTagCount, botposeAvgDistM);
        telemetry.addData("Shoot Angle Servo", "%.3f", shootAnglePos);
        telemetry.addData("Reference Row", "%.2f m -> %.3f",
                filteredHorizontalDistanceM, shootAnglePos);
        telemetry.addData("Known Reference", "%.2f m -> %.3f",
                REFERENCE_DISTANCE_M, REFERENCE_SERVO_POS);
        telemetry.addData("Distance Cal", "scale=%.4f offset=%.2f m",
                DISTANCE_SCALE, DISTANCE_OFFSET_M);
        telemetry.addData("Controls", "RS y=move, dpad up/down=0.010, left/right=0.001, A=reset");
        telemetry.update();
    }

    private void applyLimelightPipeline() {
        if (limelight == null || activePipeline == LIMELIGHT_PIPELINE) {
            return;
        }
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        activePipeline = LIMELIGHT_PIPELINE;
    }

    private double metersToInches(double meters) {
        return Double.isNaN(meters) ? Double.NaN : meters * 39.3701;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
