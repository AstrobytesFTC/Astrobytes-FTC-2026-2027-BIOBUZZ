package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Automatic Hood Test")
public class LimeLightDistance extends LinearOpMode {

    // Hardware
    private DcMotorEx phood;
    private Limelight3A limelight;

    // GEAR CONFIGURATION
    private static final double MOTOR_TEETH = 10.0;
    private static final double ARC_TEETH = 15.0;

    // Motor rotations per hood rotation
    private static final double GEAR_RATIO =
            ARC_TEETH / MOTOR_TEETH;

    // IMPORTANT:
// Replace this with the correct encoder counts per
// motor revolution once the goBILDA model is identified.
    private static final double ENCODER_TICKS_PER_REV = 537.7;

// HOOD POSITIONS (HOOD ARC TEETH)

    private static final int CLOSE = 1;
    private static final int MIDDLE = 2;
    private static final int FAR = 3;

    private static final double CLOSE_TEETH = 13;
    private static final double MIDDLE_TEETH = 9;
    private static final double FAR_TEETH = 7;

// ASSIGNMENT VELOCITIES
// Assumed to be encoder ticks per second.

    private static final double CLOSE_VELOCITY = 1250;
    private static final double MIDDLE_VELOCITY = 1200;
    private static final double FAR_VELOCITY = 1200;

// DISTANCE SETTINGS (INCHES)

    private static final double CLOSE_DISTANCE = 27;
    private static final double MIDDLE_DISTANCE = 39;
    private static final double FAR_DISTANCE = 43;

// Distance transition thresholds
// These are halfway points between assignment distances.

    private static final double CLOSE_TO_MIDDLE = 33;
    private static final double MIDDLE_TO_FAR = 41;

// VISION SETTINGS

    private static final double CAMERA_HEIGHT = 42;
    private static final double GOAL_HEIGHT = 74.95;
    private static final double CAMERA_ANGLE = 45;

    private static final double ALPHA = 0.25;
    private static final int REQUIRED_READINGS = 3;

    private static final long TARGET_LOSS_GRACE_MS = 300;

    private static final double MIN_DISTANCE = 10;
    private static final double MAX_DISTANCE = 100;

// STATE

    private double filteredDistance = 0;
    private double distance = 0;

    private int hoodPosition = FAR;
    private int pendingPosition = FAR;

    private int candidate = -1;
    private int confirmations = 0;

    private boolean moving = false;
    private boolean targetVisible = false;
    private boolean hasDistance = false;

    private long lastValidTargetTime = 0;
    private double lastTimestamp = Double.NaN;

    private int targetEncoderPosition = 0;

// DISTANCE CALCULATION

    private double getDistance(double ty) {

        if (!Double.isFinite(ty)) {
            return -1;
        }

        double angle = CAMERA_ANGLE + ty;

        if (angle <= 0 || angle >= 90) {
            return -1;
        }

        double height = GOAL_HEIGHT - CAMERA_HEIGHT;

        if (height <= 0) {
            return -1;
        }

        double result =
                (height / Math.tan(Math.toRadians(angle))) / 2.54;

        if (!Double.isFinite(result) || result <= 0) {
            return -1;
        }

        return result;
    }

// HOOD POSITION CALCULATIONS

    private double getHoodTeeth(int position) {

        switch (position) {

            case CLOSE:
                return CLOSE_TEETH;

            case MIDDLE:
                return MIDDLE_TEETH;

            case FAR:
                return FAR_TEETH;

            default:
                return FAR_TEETH;
        }
    }

    private double getVelocity(int position) {

        switch (position) {

            case CLOSE:
                return CLOSE_VELOCITY;

            case MIDDLE:
                return MIDDLE_VELOCITY;

            case FAR:
                return FAR_VELOCITY;

            default:
                return FAR_VELOCITY;
        }
    }

// Convert hood arc tooth position into motor encoder position.
//
// This assumes the tooth references correspond to evenly
// spaced positions on the same 15-tooth arc.

    private int getEncoderTarget(int position) {

        double teeth = getHoodTeeth(position);

        double hoodRotations = teeth / ARC_TEETH;

        double motorRotations =
                hoodRotations * GEAR_RATIO;

        double encoderTicks =
                motorRotations * ENCODER_TICKS_PER_REV;

        return (int) Math.round(encoderTicks);
    }

// POSITION SELECTION

    private int getDesiredPosition(double d) {

        if (hoodPosition == FAR) {

            if (d < MIDDLE_TO_FAR) {
                return MIDDLE;
            }

            return FAR;
        }

        if (hoodPosition == MIDDLE) {

            if (d < CLOSE_TO_MIDDLE) {
                return CLOSE;
            }

            if (d > MIDDLE_TO_FAR) {
                return FAR;
            }

            return MIDDLE;
        }

        if (hoodPosition == CLOSE) {

            if (d > CLOSE_TO_MIDDLE) {
                return MIDDLE;
            }

            return CLOSE;
        }

        return FAR;
    }

// POSITION CONFIRMATION

    private void updateTarget(double d) {

        if (moving) {
            return;
        }

        int desired = getDesiredPosition(d);

        if (desired == hoodPosition) {

            candidate = -1;
            confirmations = 0;

            return;
        }

        if (candidate != desired) {

            candidate = desired;
            confirmations = 1;

        } else {

            confirmations++;
        }

        if (confirmations >= REQUIRED_READINGS) {

            moveHood(desired);

            candidate = -1;
            confirmations = 0;
        }
    }

// HOOD MOVEMENT

    private void moveHood(int position) {

        if (moving || position == hoodPosition) {
            return;
        }

        if (position < CLOSE || position > FAR) {
            return;
        }

        targetEncoderPosition = getEncoderTarget(position);

        pendingPosition = position;

        phood.setTargetPosition(targetEncoderPosition);

        phood.setMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        phood.setVelocity(
                getVelocity(position)
        );

        moving = true;
    }

// MOVEMENT CONTROL

    private void updateMovement() {

        if (!moving) {
            return;
        }

        if (!phood.isBusy()) {

            phood.setVelocity(0);

            moving = false;

            hoodPosition = pendingPosition;
        }
    }

// LIMELIGHT PROCESSING

    private void updateVision() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double timestamp = result.getTimestamp();

            if (Double.isFinite(timestamp)
                    && timestamp != lastTimestamp) {

                lastTimestamp = timestamp;

                double newDistance =
                        getDistance(result.getTy());

                if (newDistance >= MIN_DISTANCE
                        && newDistance <= MAX_DISTANCE) {

                    long now = System.currentTimeMillis();

                    boolean reacquired = !targetVisible;

                    if (!hasDistance || reacquired) {

                        filteredDistance = newDistance;
                        hasDistance = true;

                    } else {

                        filteredDistance =
                                ALPHA * newDistance
                                        + (1 - ALPHA) * filteredDistance;
                    }

                    distance = filteredDistance;

                    lastValidTargetTime = now;
                    targetVisible = true;

                    if (!moving) {
                        updateTarget(distance);
                    }
                }
            }
        }

        long now = System.currentTimeMillis();

        targetVisible =
                lastValidTargetTime != 0
                        && now - lastValidTargetTime
                        <= TARGET_LOSS_GRACE_MS;

        if (!targetVisible) {

            candidate = -1;
            confirmations = 0;
        }
    }

// MAIN OPMODE

    @Override
    public void runOpMode() {

        phood = hardwareMap.get(
                DcMotorEx.class, "phood"
        );

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight"
        );

        phood.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        phood.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        phood.setTargetPosition(0);

        phood.setMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        phood.setVelocity(0);

        limelight.pipelineSwitch(0);

        telemetry.addLine("Automatic Hood Test Ready");

        telemetry.addLine(
                "Place hood at FAR starting position"
        );

        telemetry.addData("Motor Gear Teeth", MOTOR_TEETH);
        telemetry.addData("Hood Arc Teeth", ARC_TEETH);

        telemetry.addData("Position 1", "27 in / 13 teeth / 1250");
        telemetry.addData("Position 2", "39 in / 9 teeth / 1200");
        telemetry.addData("Position 3", "43 in / 7 teeth / 1200");

        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        limelight.start();

        try {

            while (opModeIsActive()) {

                updateMovement();

                updateVision();

                telemetry.addData(
                        "Target Visible", targetVisible
                );

                telemetry.addData(
                        "Distance (inches)", "%.2f", distance
                );

                telemetry.addData(
                        "Hood Position", hoodPosition
                );

                telemetry.addData(
                        "Hood Arc Teeth",
                        getHoodTeeth(hoodPosition)
                );

                telemetry.addData(
                        "Target Encoder",
                        targetEncoderPosition
                );

                telemetry.addData(
                        "Current Encoder",
                        phood.getCurrentPosition()
                );

                telemetry.addData(
                        "Target Velocity",
                        getVelocity(hoodPosition)
                );

                telemetry.addData(
                        "Moving", moving
                );

                telemetry.addData(
                        "Confirmations", confirmations
                );

                telemetry.addData(
                        "Gear Ratio", "10:15"
                );

                telemetry.update();

                idle();
            }

        } finally {

            phood.setVelocity(0);

            limelight.stop();
        }
    }
}