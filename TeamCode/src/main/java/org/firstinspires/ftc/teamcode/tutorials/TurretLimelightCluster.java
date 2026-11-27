package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretLimelightCluster {
    private Servo turretServo;

    // PD gains
    private double kP = 0.0015;
    private double kD = 0.00003;

    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.7;
    private final double MAX_STEP = 0.008;
//0.3, 0.8
    private double minPosition = 0.1;
    private double maxPosition = 0.9;
    private double currentPosition = 0.3;
    private double power = 0;
    private double offset = 0;
    private boolean firstRun = true;
    private double lastTimestamp = -1;
    private final ElapsedTime timer = new ElapsedTime();

    // ---------------- BIOBUZZ tag filtering ----------------
    // Red tags = 30-37, Blue tags = 38-45. Each CELL has a cluster of 4 tags:
    //   cluster 0 = 30-33, 1 = 34-37, 2 = 38-41, 3 = 42-45
    private static final int FIRST_TAG_ID = 30;
    private static final int TAGS_PER_CLUSTER = 4;
    private static final int NUM_CLUSTERS = 4;

    // Minimum number of tags from one cluster we must see before we trust it.
    // Higher = steadier aim, but the turret stops tracking more easily.
    private static final int MIN_TAGS_IN_CLUSTER = 2;

    // -1 means "no filter, use the Limelight's primary target" (old behavior)
    private int minTagId = -1;
    private int maxTagId = -1;

    public double getPosition() { return currentPosition; }
    public double getLastError() { return lastError; }

    public void init(HardwareMap hwMap, String servoName, double startPos) {
        turretServo = hwMap.get(Servo.class, servoName);
        currentPosition = Range.clip(startPos, minPosition, maxPosition);
        turretServo.setPosition(currentPosition);
    }

    // Choose which alliance's tags to track, e.g. (30, 37) for red, (38, 45) for blue
    public void setTargetTagRange(int min, int max) {
        minTagId = min;
        maxTagId = max;
    }

    // kD
    public void setkD(double newkD) {
        kD = newkD;
    }

    public double getkD() {
        return kD;
    }

    // kP
    public void setkP(double newkP) {
        kP = newkP;
    }

    public double getkP() {
        return kP;
    }

    public void resetTimer() {
        timer.reset();
        lastError = 0;
        firstRun = true;
        lastTimestamp = -1;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void update(LLResult result, Telemetry telemetry, String name) {
        if (result == null || !result.isValid()) {
            firstRun = true;
            return;
        }

        // ---- 1. Work out which horizontal angle (tx) to aim at ----
        double tx;
        if (minTagId < 0) {
            // No alliance filter: use whatever the Limelight picked
            tx = result.getTx();
        } else {
            double[] sumTx = new double[NUM_CLUSTERS];
            double[] sumArea = new double[NUM_CLUSTERS];
            int[] count = new int[NUM_CLUSTERS];

            // Add up every visible tag that belongs to our alliance, grouped by cluster
            for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                int id = f.getFiducialId();
                if (id < minTagId || id > maxTagId) continue;   // wrong alliance, ignore

                int cluster = (id - FIRST_TAG_ID) / TAGS_PER_CLUSTER;
                if (cluster < 0 || cluster >= NUM_CLUSTERS) continue;

                sumTx[cluster] += f.getTargetXDegrees();
                sumArea[cluster] += f.getTargetArea();
                count[cluster]++;
            }

            // Pick the cluster with the most visible tags (ties go to the larger one)
            int best = -1;
            for (int c = 0; c < NUM_CLUSTERS; c++) {
                if (count[c] == 0) continue;
                if (best < 0
                        || count[c] > count[best]
                        || (count[c] == count[best] && sumArea[c] > sumArea[best])) {
                    best = c;
                }
            }

            // Nothing usable in view: don't move the turret
            if (best < 0 || count[best] < MIN_TAGS_IN_CLUSTER) {
                firstRun = true;
                return;
            }

            tx = sumTx[best] / count[best];   // average angle of the cluster's tags
            telemetry.addData(name + " Cluster", best + " (" + count[best] + " tags)");
        }

        // ---- 2. Skip duplicate frames ----
        double ts = result.getTimestamp();
        if (ts == lastTimestamp) return;   // same frame, don't move again
        lastTimestamp = ts;

        double deltaTime = timer.seconds();
        timer.reset();

        // ---- 3. PD controller ----
        double adjustedTx = tx + offset;
        double error = goalX - adjustedTx;
        double pTerm = error * kP;
        double dTerm = 0;

        if (!firstRun && deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }
        lastError = error;
        firstRun = false;

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_STEP, MAX_STEP);
        }

        // safety encoder check or magnetic limit switch check or other safeties

        // ---- 4. Move the servo a small step, kept inside its limits ----
        currentPosition = Range.clip(currentPosition + power, minPosition, maxPosition);
        turretServo.setPosition(currentPosition);

        telemetry.addData(name + " Error", error);
        telemetry.addData(name + " P Term", pTerm);
        telemetry.addData(name + " D Term", dTerm);
        telemetry.addData(name + " Power", power);
        telemetry.addData(name + " Servo Position", currentPosition);
        telemetry.addData(name + " Delta Time", deltaTime);
    }

    public void stop() {
    }
}