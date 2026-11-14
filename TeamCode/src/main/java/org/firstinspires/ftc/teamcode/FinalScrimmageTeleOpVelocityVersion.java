package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.tutorials.TurretLimelightNew;

@TeleOp
public class FinalScrimmageTeleOpVelocityVersion extends LinearOpMode {
    private static final double NORMAL_SPEED = 0.5;
    private static final double TURBO_SPEED = 1.0;
    private static final double STRAFE_CORRECTION = 1.1;

    private static final boolean INVERT_ALL_DRIVE = false;

    private static final double VELOCITY_STEP = 50;
    private static final double MIN_VELOCITY = 0;
    private static final double MAX_VELOCITY = 2800;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private DcMotor intake;

    private DcMotorEx feeder;

    private DcMotorEx pollenTurret;
    private DcMotorEx nectarTurret;

    private Servo nectarServo;
    private Servo pollenServo;
    private CRServo flowerServo;

    private double pollenPos = 0.5;
    private double nectarPos = 0.5;

    private Limelight3A limelight;
    private TurretLimelightNew pollenAligner = new TurretLimelightNew();
    private TurretLimelightNew nectarAligner = new TurretLimelightNew();

    double targetVelocity = 2000;

    boolean lastUp, lastDown;

    final double TURRET_RANGE_DEG = 360;

    double aim(double x, double y, double current) {
        if (Math.hypot(x, y) < 0.5) return current;
        return Range.clip(0.5 + Math.toDegrees(Math.atan2(x, -y)) / TURRET_RANGE_DEG, 0, 1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("fL");
        backLeftMotor = hardwareMap.dcMotor.get("bL");
        frontRightMotor = hardwareMap.dcMotor.get("fR");
        backRightMotor = hardwareMap.dcMotor.get("bR");
        intake = hardwareMap.dcMotor.get("intake");
        nectarTurret = hardwareMap.get(DcMotorEx.class, "nt");

        pollenTurret = hardwareMap.get(DcMotorEx.class, "pt");
        nectarServo = hardwareMap.get(Servo.class, "ns");
        pollenServo = hardwareMap.get(Servo.class, "pss");
        flowerServo = hardwareMap.get(CRServo.class, "fs");

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] drive = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};

        for (DcMotor m : drive) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pollenTurret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        nectarTurret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pollenTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pollenAligner.init(hardwareMap, "pss");
        nectarAligner.init(hardwareMap, "ns");
        limelight.pipelineSwitch(0);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        telemetry.addLine("Ready. Options resets heading.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        pollenAligner.resetTimer();
        nectarAligner.resetTimer();
        limelight.start();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }
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

            intake.setPower((gamepad1.right_trigger - gamepad1.left_trigger));

            if (gamepad2.dpad_up && !lastUp) targetVelocity += VELOCITY_STEP;
            if (gamepad2.dpad_down && !lastDown) targetVelocity -= VELOCITY_STEP;
            targetVelocity = Range.clip(targetVelocity, MIN_VELOCITY, MAX_VELOCITY);

            lastUp = gamepad2.dpad_up;
            lastDown = gamepad2.dpad_down;

            if (gamepad2.a) {
                pollenTurret.setVelocity(targetVelocity);
            } else {
                pollenTurret.setVelocity(0);
                nectarTurret.setVelocity(0);
            }
            if(gamepad2.b){
                nectarTurret.setVelocity(targetVelocity);
            }

            feeder.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            LLResult result = limelight.getLatestResult();
            pollenAligner.update(result, telemetry, "Pollen");
            nectarAligner.update(result, telemetry, "Nectar");

            if (result != null && result.isValid()) {
                telemetry.addData("Target TX", result.getTx());
                telemetry.addData("Target TY", result.getTy());
            } else {
                telemetry.addLine("No Tag Detected.");
            }

            if(gamepad2.dpad_left){
                flowerServo.setPower(-1);
                sleep(500);
                flowerServo.setPower(0);
            }else if(gamepad2.dpad_right){
                flowerServo.setPower(1);
                sleep(500);
                flowerServo.setPower(0);
            }
            telemetry.addData("Target velocity", targetVelocity);
            telemetry.addData("Pollen Turret velocity", pollenTurret.getVelocity());
            telemetry.addData("Pollen amps", pollenTurret.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Battery V", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addData("Pollen pos", pollenServo.getPosition());
            telemetry.addData("Nectar pos", nectarServo.getPosition());
            telemetry.addData("Flower pos", flowerServo.getPower());
            telemetry.addLine("CONTROLS:");
            telemetry.addLine("A - Pollen turret");
            telemetry.addLine("B - Nectar turret");
            telemetry.addLine("X - Feeder");
            telemetry.addLine("Y - Intake");
            telemetry.addLine("DPAD LEFT - Flower servo down");
            telemetry.addLine("DPAD RIGHT - Flower servo up");
            telemetry.addLine("DPAD UP/DOWN - Turret velocity adjuster");
            telemetry.addLine("BACK - Stop Turrets");
            telemetry.addLine("Right Bumper - Turbo Mode/Speed");
            telemetry.addLine("Limelight auto-align is always on");
            telemetry.addLine("MOVEMENT & INTAKE GAMEPAD 1");
            telemetry.addLine("FEEDER & EVT ELSE");
            telemetry.addData("Pollen pos", pollenTurret.getCurrentPosition());
            telemetry.addData("Pollen vel", pollenTurret.getVelocity());
            telemetry.addData("Pollen power", pollenTurret.getPower());
            telemetry.addData("Nectar pos", nectarTurret.getCurrentPosition());
            telemetry.addData("Nectar vel", nectarTurret.getVelocity());
            telemetry.addData("Nectar power", nectarTurret.getPower());
            telemetry.update();
        }

        pollenAligner.stop();
        nectarAligner.stop();
        limelight.stop();
    }
}