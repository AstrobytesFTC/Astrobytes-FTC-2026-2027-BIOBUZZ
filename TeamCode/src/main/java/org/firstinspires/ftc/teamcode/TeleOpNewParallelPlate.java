package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "NewRobotTeleOp", group = "Main")
public class TeleOpNewParallelPlate extends LinearOpMode {

    // Tuning knobs
    private static final double NORMAL_SPEED = 0.5;  // same as your old code
    private static final double TURBO_SPEED = 1.0;   // hold right bumper
    private static final double STRAFE_CORRECTION = 1.1;

    // Flip to true if the robot drives backward when you push forward
    private static final boolean INVERT_ALL_DRIVE = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("bR");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

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

//            double botHeading = aqimu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
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

            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("FL/BL", "%.2f / %.2f", flP, blP);
            telemetry.addData("FR/BR", "%.2f / %.2f", frP, brP);
            telemetry.update();
        }
    }
}