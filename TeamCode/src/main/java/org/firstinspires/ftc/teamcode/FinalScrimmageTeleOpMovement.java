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

// Needed only when the pollen turret / limelight code is re-enabled:
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.hardware.limelightvision.LLResult;
// import com.qualcomm.hardware.limelightvision.Limelight3A;
// import org.firstinspires.ftc.teamcode.tutorials.TurretLimelightNew;

@TeleOp
public class FinalScrimmageTeleOpMovement extends LinearOpMode {

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


    // Disabled hardware (nectar turret, servos, limelight)
     private DcMotorEx nectarTurret;
    // private Servo nectarServo;
    // private Servo pollenServo;
    // private CRServo flowerServo;
    // private Limelight3A limelight;
    // private TurretLimelightNew pollenAligner = new TurretLimelightNew();
    // private TurretLimelightNew nectarAligner = new TurretLimelightNew();

    // ---------------- State ----------------
    private double targetVelocity = 2000;
    private boolean lastUp, lastDown;

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
        flowerServo = hardwareMap.get(CRServo.class, "fs");


         nectarTurret = hardwareMap.get(DcMotorEx.class, "nt");
        // nectarServo = hardwareMap.get(Servo.class, "ns");
        // pollenServo = hardwareMap.get(Servo.class, "pss");
        // flowerServo = hardwareMap.get(CRServo.class, "fs");
        // limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pollenTurret.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------------- Drive setup ----------------
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        DcMotor[] drive = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};
        for (DcMotor m : drive) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---------------- Turret setup ----------------
        pollenTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        pollenTurret.setDirection(DcMotorSimple.Direction.REVERSE);

        // pollenTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // ---------------- Vision setup (disabled) ----------------
        // pollenAligner.init(hardwareMap, "pss");
        // nectarAligner.init(hardwareMap, "ns");
        // limelight.pipelineSwitch(0);

        // ---------------- IMU ----------------
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // pollenAligner.resetTimer();
        // nectarAligner.resetTimer();
        // limelight.start();

        while (opModeIsActive()) {

            // ---------------- Drive (robot-centric) ----------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Field-centric is disabled (botHeading stays 0). To enable, uncomment the reset
            // below and set botHeading from imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS).
            // if (gamepad1.options) {
            //     imu.resetYaw();
            // }
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
            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            // NOTE: right_bumper is also turbo above, so reversing the feeder also turbos.
            if (gamepad2.left_bumper) {
                feeder.setPower(1);
            } else if (gamepad2.right_bumper) {
                feeder.setPower(-1);
            } else {
                feeder.setPower(0);
            }

            // ---------------- Turret velocity ----------------
            if (gamepad2.dpad_up && !lastUp) targetVelocity += VELOCITY_STEP;
            if (gamepad2.dpad_down && !lastDown) targetVelocity -= VELOCITY_STEP;
            targetVelocity = Range.clip(targetVelocity, MIN_VELOCITY, MAX_VELOCITY);
            lastUp = gamepad2.dpad_up;
            lastDown = gamepad2.dpad_down;

            if (gamepad2.a) {
                pollenTurret.setVelocity(targetVelocity);
            } else {
                pollenTurret.setVelocity(0);
            }


             if (gamepad2.b) {
                 nectarTurret.setVelocity(targetVelocity);
             } else {
                 nectarTurret.setVelocity(0);
             }

             if(gamepad2.x){
                 flowerServo.setPower(1);
                 sleep(500);
                 flowerServo.setPower(0);
             }

             if(gamepad2.y){
                 flowerServo.setPower(-1);
                 sleep(500);
                 flowerServo.setPower(0);
             }

            // ---------------- Telemetry ----------------
            telemetry.addData("Target velocity", targetVelocity);
            telemetry.addData("Gamepad2.A pressed", gamepad2.a);
            telemetry.addData("Pollen pos", pollenTurret.getCurrentPosition());
            telemetry.addData("Pollen vel", pollenTurret.getVelocity());
            telemetry.addData("Nectar vel", nectarTurret.getVelocity());
            telemetry.addData("Pollen power", pollenTurret.getPower());
            telemetry.update();
        }

        // ---------------- Shutdown ----------------
        pollenTurret.setVelocity(0);
        pollenTurret.setPower(0);
        // pollenAligner.stop();
        // nectarAligner.stop();
        // limelight.stop();
    }
}