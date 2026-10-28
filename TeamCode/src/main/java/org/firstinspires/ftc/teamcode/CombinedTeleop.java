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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "CombinedTeleop", group = "Main")
public class CombinedTeleop extends LinearOpMode {
    // Tuning knobs
    private static final double NORMAL_SPEED = 0.5; // same as your old code
    private static final double TURBO_SPEED = 1.0; // hold right bumper
    private static final double STRAFE_CORRECTION = 1.1;

    // Flip to true if the robot drives backward when you push forward
    private static final boolean INVERT_ALL_DRIVE = false;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private DcMotor intake;

    private DcMotorEx pollenTurret;
    private DcMotorEx feeder;

    private Servo nectarServo;
    private Servo pollenServo;
    private CRServo flowerServo;

    private double pollenPos = 0.5;
    private double nectarPos = 0.5;

    double targetVelocity = 1000;

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

        pollenTurret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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

            // double botHeading = aqimu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            //robot centric drivetrain rn
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
            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            //pollen turret velocity
            if (gamepad2.dpad_up && !lastUp) targetVelocity += 50;
            if (gamepad2.dpad_down && !lastDown) targetVelocity -= 50;

            lastUp = gamepad2.dpad_up;
            lastDown = gamepad2.dpad_down;

            // Test controls
            if (gamepad2.a) pollenTurret.setVelocity(targetVelocity);
            // if (gamepad1.b) nectarTurret.setVelocity(targetVelocity);

            if (gamepad2.x) pollenTurret.setVelocity(-targetVelocity);
            // if (gamepad1.y) nectarTurret.setVelocity(-targetVelocity);

            if (gamepad2.back) {
                pollenTurret.setVelocity(0);
                // nectarTurret.setVelocity(0);
            }

            //Feeder!   REVERSE THE ORDER IF FEEDER AND INTAKE MUST RUN IN DIFFERENT DIRECTION
            //OR MAP CONTROL FOR FEEDER DIFFERENTLY IF THEY RUN INDEPENDENTLY

            //rn, feeder and intake are mapped to the same buttons and directions, change as needed
            feeder.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            //Rotation
            pollenPos = aim(gamepad2.left_stick_x, gamepad2.left_stick_y, pollenPos);
            nectarPos = aim(gamepad2.right_stick_x, gamepad2.right_stick_y, nectarPos);

            pollenServo.setPosition(pollenPos);
            nectarServo.setPosition(nectarPos);

            //flower continious rotation servo or sm
            if(gamepad2.dpad_left) {
                flowerServo.setPower(-1);

            }else if(gamepad2.dpad_right) {
                flowerServo.setPower(1);
            }else{
                flowerServo.setPower(0);
            }

            //Telemetry
            telemetry.addData("Target", targetVelocity);
            telemetry.addData("Pollen Turret velocity", pollenTurret.getVelocity());
            telemetry.addData("Pollen amps", pollenTurret.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Battery V", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addData("Pollen pos", pollenServo.getPosition());
            telemetry.addData("Nectar pos", nectarServo.getPosition());
            telemetry.addData("Flower pos", flowerServo.getPower());


            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("FL/BL", "%.2f / %.2f", flP, blP);
            telemetry.addData("FR/BR", "%.2f / %.2f", frP, brP);
            telemetry.update();
        }
    }
}
