package org.firstinspires.ftc.teamcode;


// RR-specific imports
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Autonomous(name = "Scrimmage Auto Red Easy", group = "Autonomous")
public class ScrimmageAutoRedEasy extends LinearOpMode {


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


    double velocityPowerFar = 1690;
    double velocityPowerNear = 1350;
    // lift class
    private boolean initialized = false;


    public void runOpMode() {


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

//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] driveMotors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};
        for (DcMotor m : driveMotors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ---------------- Turret setup ----------------
        pollenTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollenTurret.setDirection(DcMotorSimple.Direction.REVERSE);
//        pollenTurret.setDirection(DcMotorSimple.Direction.REVERSE);

        Pose2d beginPose = new Pose2d(new Vector2d(-63,-60), Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();

        Action Scrimmage2Auto = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-50,-60))

                .build();


        Actions.runBlocking(new SequentialAction(Scrimmage2Auto));

        telemetry.update();



    }

}