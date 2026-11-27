package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.tutorials.TurretLimelightCluster;

// No @TeleOp here: this is a shared base class. The Red and Blue classes are the real OpModes.
public abstract class Qual1Base extends LinearOpMode {

    // Each alliance class says which AprilTag IDs to track
    protected abstract int getMinTagId();
    protected abstract int getMaxTagId();

    // ---------------- Constants ----------------
    private static final double NORMAL_SPEED = 0.5;
    private static final double TURBO_SPEED = 1.0;
    private static final double STRAFE_CORRECTION = 1.1;
    private static final boolean INVERT_ALL_DRIVE = false;

    private static final double VELOCITY_STEP = 50;
    private static final double MIN_VELOCITY = 0;
    private static final double MAX_VELOCITY = 2800;

    private static final double TURRET_RANGE_DEG = 360;

    // ---------------- Hardware ----------------
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private DcMotor intake;
    private DcMotorEx feeder;
    private DcMotorEx pollenTurret;

    private Servo pollenServo;
    private Servo nectarServo;
    private CRServo flowerServo;

    private DcMotorEx nectarTurret;
    private Limelight3A limelight;
    private TurretLimelightCluster pollenAligner = new TurretLimelightCluster();
    private TurretLimelightCluster nectarAligner = new TurretLimelightCluster();

    // ---------------- State ----------------
    private double targetVelocity = 2000;
    private boolean lastUp, lastDown;

    // Limelight auto-align (gamepad2 dpad left = on, dpad right = off)
    private boolean alignOn = false;

    // Currently unused (servo aiming)
    private double pollenPos = 0.5;
    private double nectarPos = 0.5;
    private static final double POLLEN_FORWARD = 0.3;
    private static final double NECTAR_FORWARD = 0.5;

    private double aim(double x, double y, double current) {
        if (Math.hypot(x, y) < 0.5) return current;
        return Range.clip(0.5 + Math.toDegrees(Math.atan2(x, -y)) / TURRET_RANGE_DEG, 0, 1);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- Hardware map ----------------
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("br");
        intake = hardwareMap.dcMotor.get("intake");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        pollenTurret = hardwareMap.get(DcMotorEx.class, "pt");
        pollenServo = hardwareMap.get(Servo.class, "ps");
        nectarServo = hardwareMap.get(Servo.class, "ns");
        nectarTurret = hardwareMap.get(DcMotorEx.class, "nt");
        flowerServo = hardwareMap.get(CRServo.class, "fs");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");



        // ---------------- Drive setup ----------------
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] drive = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};
        for (DcMotor m : drive) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ---------------- Turret setup ----------------
        pollenTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollenTurret.setDirection(DcMotorSimple.Direction.REVERSE);

        nectarTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nectarTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------------- Vision setup ----------------
        pollenAligner.init(hardwareMap, "ps", POLLEN_FORWARD);
        nectarAligner.init(hardwareMap, "ns", NECTAR_FORWARD);


// Pollen tuning
        pollenAligner.setkP(0.0015);
        pollenAligner.setkD(0.00003);

// Nectar tuning
        nectarAligner.setkP(0.0010);
        nectarAligner.setkD(0.00003);

// Offsets
        pollenAligner.setOffset(0.0);
        nectarAligner.setOffset(0.0);


        // Tell both aligners which alliance's tags to track (set by the Red/Blue class)
        pollenAligner.setTargetTagRange(getMinTagId(), getMaxTagId());
        nectarAligner.setTargetTagRange(getMinTagId(), getMaxTagId());

        limelight.pipelineSwitch(0);   // pipeline 0 must be an AprilTag pipeline

        // ---------------- IMU ----------------
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        telemetry.addLine("good luck!");
        telemetry.addData("Target tags", getMinTagId() + " - " + getMaxTagId());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        pollenAligner.resetTimer();
        nectarAligner.resetTimer();
        limelight.start();

        while (opModeIsActive()) {

            // ---------------- Drive (robot-centric) ----------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Field-centric is disabled (botHeading stays 0).
            double botHeading = 0;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= STRAFE_CORRECTION;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double flP = (rotY + rotX + rx) / denominator;
            double blP = (rotY - rotX + rx) / denominator;
            double frP = (rotY - rotX - rx) / denominator;
            double brP = (rotY + rotX - rx) / denominator;

            double scale = gamepad1.right_bumper ? TURBO_SPEED : NORMAL_SPEED;
            if (INVERT_ALL_DRIVE) scale = -scale;

            frontLeftMotor.setPower(flP * scale);
            backLeftMotor.setPower(blP * scale);
            frontRightMotor.setPower(frP * scale);
            backRightMotor.setPower(brP * scale);

            // ---------------- Intake and feeder ----------------
            intake.setPower((gamepad1.left_trigger - gamepad1.right_trigger));

            if (gamepad2.left_bumper) {
                feeder.setPower(1);
            } else if (gamepad2.right_bumper) {
                feeder.setPower(-1);
            } else {
                feeder.setPower(0);
            }

            // ---------------- Turret velocity ----------------
            if (gamepad2.a) {
                pollenTurret.setVelocity(1250);
            } else if (gamepad2.b) {
                nectarTurret.setVelocity(1750);
            } else if (gamepad2.x) {
                pollenTurret.setVelocity(1300);
            } else if (gamepad2.y) {
                nectarTurret.setVelocity(1800);
            } else if (gamepad2.back) {
                nectarTurret.setVelocity(0);
                pollenTurret.setVelocity(0);
            }

            if (gamepad2.dpad_up) {
                flowerServo.setPower(1);
                sleep(500);
                flowerServo.setPower(0);
            }

            if (gamepad2.dpad_down) {
                flowerServo.setPower(-1);
                sleep(500);
                flowerServo.setPower(0);
            }

            // ---------------- Limelight auto-align (dpad left = on, dpad right = off) ----------------
            if (gamepad2.dpad_left) alignOn = true;
            if (gamepad2.dpad_right) alignOn = false;

            LLResult result = limelight.getLatestResult();
            if (alignOn) {
                pollenAligner.update(result, telemetry, "Pollen");
                nectarAligner.update(result, telemetry, "Nectar");
            } else {
                pollenAligner.resetTimer();
                nectarAligner.resetTimer();
            }

            // ---------------- Telemetry ----------------
            telemetry.addData("Target tags", getMinTagId() + " - " + getMaxTagId());
            telemetry.addData("Auto-align", alignOn ? "ON" : "OFF");
            telemetry.addData("Gamepad2.A pressed", gamepad2.a);
            telemetry.addData("Pollen pos", pollenTurret.getCurrentPosition());
            telemetry.addData("Pollen vel", pollenTurret.getVelocity());
            telemetry.addData("Nectar vel", nectarTurret.getVelocity());
            telemetry.addData("Pollen power", pollenTurret.getPower());
            telemetry.addLine("A - pollen 1250");
            telemetry.addLine("B - nectar 1750");
            telemetry.addLine("X - pollen 1300");
            telemetry.addLine("Y - nectar 1800");
            telemetry.addData("Pollen servo pos", pollenAligner.getPosition());
            telemetry.addData("Nectar servo pos", nectarAligner.getPosition());
            telemetry.addData("Pollen error", pollenAligner.getLastError());
            telemetry.addData("Nectar error", nectarAligner.getLastError());
            telemetry.addData("Tags seen", result != null ? result.getFiducialResults().size() : 0);

            telemetry.update();
        }

        // ---------------- Shutdown ----------------
        pollenTurret.setVelocity(0);
        pollenTurret.setPower(0);
        nectarTurret.setVelocity(0);
        nectarTurret.setPower(0);
        pollenAligner.stop();
        nectarAligner.stop();
        limelight.stop();
    }
}