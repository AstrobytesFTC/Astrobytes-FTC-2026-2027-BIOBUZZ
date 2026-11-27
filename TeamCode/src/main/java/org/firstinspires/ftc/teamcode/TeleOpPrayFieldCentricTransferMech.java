package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOpPrayFieldCentricTransferMech extends LinearOpMode {

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
    private DcMotorEx nectarTurret;

    private Servo pollenServo;
    private Servo nectarServo;
    private CRServo flowerServo;
    private CRServo flowerServo2;

    // ---------------- State ----------------
    private double targetVelocity = 2000;
    private double polTargetVel = 1250;
    private double necTargetVel= 1600;

    // Non-blocking timer for flower servo
    private final ElapsedTime flowerTimer = new ElapsedTime();
    private boolean flowerRunning = false;

    // Currently unused (servo aiming)
    private double pollenPos = 0.5;
    private double nectarPos = 0.5;

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
        flowerServo2 = hardwareMap.get(CRServo.class, "fs2");

        // ---------------- Drive setup ----------------
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pollenTurret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(13.6, 10.0, 0.0, 0.0));
        nectarTurret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(14.3, 15.0, 0.0, 0.0));

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

        // ---------------- IMU ----------------
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));
        imu.resetYaw();

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        double headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        while (opModeIsActive()) {

            // ---------------- Drive (field-centric) ----------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if (gamepad1.options || gamepad1.back) {
                headingOffset = rawYaw;
            }
            double botHeading = AngleUnit.normalizeRadians(rawYaw - headingOffset);

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

            // ---------------- Intake ----------------
            double intakePower = gamepad1.left_trigger - gamepad1.right_trigger;
            intake.setPower(intakePower*0.85);

            // ---------------- Turrets ----------------
            if (gamepad2.a) {
                pollenTurret.setVelocity(1250);
            } else {
                pollenTurret.setVelocity(0);
            }

            if (gamepad2.b) {
                nectarTurret.setVelocity(1600);
            } else {
                nectarTurret.setVelocity(0);
            }

            if (gamepad2.back) {
                pollenTurret.setVelocity(0);
                nectarTurret.setVelocity(0);
            }

            // ---------------- Simultaneous Dual Flower Servos ----------------
            if (gamepad2.dpad_up) {
                flowerServo.setPower(1.0);
                flowerServo2.setPower(1.0);
                flowerTimer.reset();
                flowerRunning = true;
            } else if (gamepad2.dpad_down) {
                flowerServo.setPower(-1.0);
                flowerServo2.setPower(-1.0);
                flowerTimer.reset();
                flowerRunning = true;
            } else if (!flowerRunning || flowerTimer.milliseconds() >= 300) {
                flowerServo.setPower(0.0);
                flowerServo2.setPower(0.0);
                flowerRunning = false;
            }

            // ---------------- Feeder Control ----------------
            double pollenVel = pollenTurret.getVelocity();
            double nectarVel = nectarTurret.getVelocity();

            boolean pollenOnTarget = gamepad2.a && (Math.abs(pollenVel - polTargetVel) <= polTargetVel*.1);
            boolean nectarOnTarget = gamepad2.b && (Math.abs(nectarVel - necTargetVel) <= necTargetVel*.1);

            boolean autoFeed = false;
            if (gamepad2.a && gamepad2.b) {
                autoFeed = pollenOnTarget && nectarOnTarget;
            } else if (gamepad2.a) {
                autoFeed = pollenOnTarget;
            } else if (gamepad2.b) {
                autoFeed = nectarOnTarget;
            }

            if (gamepad2.left_bumper) {
                feeder.setPower(1.0);   // Manual feed forward
            } else if (gamepad2.right_bumper) {
                feeder.setPower(-0.65); // Slow reverse transfer
            } else if (autoFeed) {
                feeder.setPower(1.0);   // Auto feed on target speed
            } else if (Math.abs(intakePower) > 0.05) {
                feeder.setPower(intakePower > 0 ? -0.85 : 0.85); // Increased feeder speed when intake is running
            } else {
                feeder.setPower(0.0);
            }

            // ---------------- Telemetry ----------------
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Pollen vel", pollenTurret.getVelocity());
            telemetry.addData("Nectar vel", nectarTurret.getVelocity());
            telemetry.addData("Feeder power", feeder.getPower());
            telemetry.addData("flowerServo Pos", flowerServo.getDirection());
            telemetry.addData("flowerServo2 Pos", flowerServo2.getDirection());
            telemetry.update();
        }

        // ---------------- Shutdown ----------------
        pollenTurret.setVelocity(0);
        pollenTurret.setPower(0);
        nectarTurret.setVelocity(0);
        nectarTurret.setPower(0);
        flowerServo.setPower(0);
        flowerServo2.setPower(0);
    }
}