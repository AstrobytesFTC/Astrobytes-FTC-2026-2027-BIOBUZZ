package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Scrimmage Auto Blue Sideee", group = "Autonomous")
public class ScrimmageAutoBlueSide extends LinearOpMode {

    private DcMotor intake;
    private DcMotorEx feeder;
    private DcMotorEx pollenTurret;
    private DcMotorEx nectarTurret;

    private Servo pollenServo;
    private Servo nectarServo;
    private CRServo flowerServo;

    public static double velocityPowerFar = 1300;
    public static double velocityPowerNear = 1400;

    public class frontShot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pollenTurret.setVelocity(velocityPowerNear);
            return false;
        }
    }

    public class stopShot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pollenTurret.setVelocity(0);
            return false;
        }
    }

    public class feedIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            feeder.setPower(1);
            return false;
        }
    }

    public class stopFeed implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            feeder.setPower(0);
            return false;
        }
    }

    @Override
    public void runOpMode() {
        intake = hardwareMap.dcMotor.get("intake");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        pollenTurret = hardwareMap.get(DcMotorEx.class, "pt");
        nectarTurret = hardwareMap.get(DcMotorEx.class, "nt");

        pollenServo = hardwareMap.get(Servo.class, "ps");
        nectarServo = hardwareMap.get(Servo.class, "ns");
        flowerServo = hardwareMap.get(CRServo.class, "fs");

        // ---------------- Turret setup ----------------
        pollenTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollenTurret.setDirection(DcMotorSimple.Direction.REVERSE);

        Pose2d beginPose = new Pose2d(new Vector2d(-63, 16), Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Action Scrimmage2Auto = drive.actionBuilder(beginPose)
                .lineToX(-59)
                .stopAndAdd(new SequentialAction(
                        new frontShot(),
                        new SleepAction(2.5),
                        // First feed
                        new feedIn(),
                        new SleepAction(0.4),
                        new stopFeed(),
                        new SleepAction(1.5), // Pause for flywheel RPM recovery
                        // Second feed
                        new feedIn(),
                        new SleepAction(0.4),
                        new stopFeed(),
                        new SleepAction(1.5),
                        // Third feed
                        new feedIn(),
                        new SleepAction(0.4),
                        new stopFeed(),
                        new SleepAction(1.5)
//                        // Fourth feed
//                        new feedIn(),
//                        new SleepAction(0.75),
//                        new stopFeed(),
//                        // Stop shooter
//                        new stopShot()
                ))
                .strafeTo(new Vector2d(-21, 52))
                .strafeTo(new Vector2d(30, 59))
                .build();

        Actions.runBlocking(Scrimmage2Auto);
    }
}
