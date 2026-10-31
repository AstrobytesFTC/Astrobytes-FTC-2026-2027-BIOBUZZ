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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class FinalScrimmageTeleOpVelocityVersion extends LinearOpMode {
    // Tuning knobs
    private static final double NORMAL_SPEED = 0.5; // same as your old code
    private static final double TURBO_SPEED = 1.0; // hold right bumper
    private static final double STRAFE_CORRECTION = 1.1;

    // Flip to true if the robot drives backward when you push forward
    private static final boolean INVERT_ALL_DRIVE = false;

    // Turret velocity tuning (ticks per second)
    private static final double VELOCITY_STEP = 25;
    private static final double MIN_VELOCITY = 0;
    private static final double MAX_VELOCITY = 2800; // set to your motor's max ticks/sec

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private DcMotor intake;

    private DcMotorEx pollenTurret;
    private DcMotorEx nectarTurret;
    private DcMotorEx feeder;

    private Servo nectarServo;
    private Servo pollenServo;
    private CRServo flowerServo;

    private double pollenPos = 0.5;
    private double nectarPos = 0.5;

    // Target turret velocity in ticks per second (replaces old 0.7 power)
    double targetVelocity = 100;

    boolean lastUp, lastDown;

    final double TURRET_RANGE_DEG = 360; // turret degrees covered by servo 0 to 1

    double aim(double x, double y, double current) {
        if (Math.hypot(x, y) < 0.5) return current; // stick released, hold position
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

        // Directions kept from your old code (all four reversed)
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

        // Velocity control needs the encoders running
        pollenTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nectarTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pollenTurret.setDirection(DcMotorSimple.Direction.REVERSE);


        // Hub orientation kept from your old code
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        telemetry.addLine("Ready. Options resets heading.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }
            // WARNING: botheading must be set at 0, that's the only way we
            // can actually run the drivetrain controls we want, I tested this.
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

            //intake!
            intake.setPower((gamepad1.right_trigger - gamepad1.left_trigger)*0.7);

            // Adjust target velocity
            if (gamepad2.dpad_up && !lastUp) targetVelocity += VELOCITY_STEP;
            if (gamepad2.dpad_down && !lastDown) targetVelocity -= VELOCITY_STEP;
            targetVelocity = Range.clip(targetVelocity, MIN_VELOCITY, MAX_VELOCITY);

            lastUp = gamepad2.dpad_up;
            lastDown = gamepad2.dpad_down;

            // Turret controls (velocity in ticks/sec)
            if (gamepad2.a) {
                pollenTurret.setVelocity(targetVelocity);
            } else if (gamepad2.back) {
                pollenTurret.setVelocity(0);
                nectarTurret.setVelocity(0);
            } else if (gamepad2.b){
                nectarTurret.setVelocity(targetVelocity);
            }

            //Feeder Control
            feeder.setPower(gamepad2.right_trigger - gamepad2.left_trigger);


            //flower continuous rotation servo or sm
            if(gamepad2.dpad_left){
                flowerServo.setPower(-1);
                sleep(500);
                flowerServo.setPower(0);
            }else if(gamepad2.dpad_right){
                flowerServo.setPower(1);
                sleep(500);
                flowerServo.setPower(0);
            }
            //Telemetry
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
    }
}