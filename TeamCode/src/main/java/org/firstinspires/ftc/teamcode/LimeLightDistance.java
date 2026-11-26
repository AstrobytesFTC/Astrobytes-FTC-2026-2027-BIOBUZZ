package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Automatic Hood Test")
public class LimeLightDistance extends LinearOpMode {

    // Hardware
    private Servo phood;
    private Limelight3A limelight;

    // GEAR CONFIGURATION
    private static final double MOTOR_TEETH = 10.0;   // pinion on the servo
    private static final double ARC_TEETH = 15.0;

    // SERVO SETTINGS
    // Total travel of your servo in degrees (300 = standard, 1800 = goBILDA 5-turn)
    private static final double SERVO_RANGE_DEGREES = 1800;

    // Adjust if the hood doesn't line up with servo position 0.0
    private static final double SERVO_ZERO_OFFSET = 0.0;

    // Time to wait for the servo to reach its position (no feedback available)
    private static final long MOVE_TIME_MS = 600;

    // HOOD POSITIONS (HOOD ARC TEETH)
    private static final int CLOSE = 1;
    private static final int MIDDLE = 2;
    private static final int FAR = 3;

    private static final double CLOSE_TEETH = 13;
    private static final double MIDDLE_TEETH = 9;
    private static final double FAR_TEETH = 7;

    // DISTANCE SETTINGS (INCHES)
    private static final double CLOSE_DISTANCE = 27;
    private static final double MIDDLE_DISTANCE = 39;
    private static final double FAR_DISTANCE = 43;

    // Distance transition thresholds (halfway points between assignment distances)
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

    private double targetServoPosition = 0;
    private long moveStartTime = 0;

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

    // Converts hood arc teeth into a 0..1 servo position.
    // Pinion revolutions = arc teeth moved / pinion teeth
    private double getServoPosition(int position) {

        double teeth = getHoodTeeth(position);

        double servoRevs = teeth / MOTOR_TEETH;
        double servoDegrees = servoRevs * 360.0;

        return Range.clip(
                servoDegrees / SERVO_RANGE_DEGREES + SERVO_ZERO_OFFSET,
                0.0, 1.0);
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

        targetServoPosition = getServoPosition(position);
        pendingPosition = position;

        phood.setPosition(targetServoPosition);

        moveStartTime = System.currentTimeMillis();
        moving = true;
    }

    // MOVEMENT CONTROL (timer based, since servos give no feedback)
    private void updateMovement() {

        if (!moving) {
            return;
        }

        if (System.currentTimeMillis() - moveStartTime >= MOVE_TIME_MS) {
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

        phood = hardwareMap.get(Servo.class, "phood");

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight"
        );

        // Start at FAR, matching the "place hood at FAR" instruction
        targetServoPosition = getServoPosition(FAR);
        phood.setPosition(targetServoPosition);

        limelight.pipelineSwitch(0);

        telemetry.addLine("Automatic Hood Test Ready");

        telemetry.addLine(
                "Place hood at FAR starting position"
        );

        telemetry.addData("Pinion Teeth", MOTOR_TEETH);
        telemetry.addData("Hood Arc Teeth", ARC_TEETH);

        telemetry.addData("Position 1", "27 in / 13 teeth");
        telemetry.addData("Position 2", "39 in / 9 teeth");
        telemetry.addData("Position 3", "43 in / 7 teeth");

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

                telemetry.addData("Target Visible", targetVisible);

                telemetry.addData("Distance (inches)", "%.2f", distance);

                telemetry.addData("Hood Position", hoodPosition);

                telemetry.addData("Hood Arc Teeth", getHoodTeeth(hoodPosition));

                telemetry.addData("Target Servo Pos", "%.3f", targetServoPosition);

                telemetry.addData("Commanded Servo Pos", "%.3f", phood.getPosition());

                telemetry.addData("Moving", moving);

                telemetry.addData("Confirmations", confirmations);

                telemetry.addData("Gear Ratio", "10:15");

                telemetry.update();

                idle();
            }

        } finally {

            limelight.stop();
        }
    }
}