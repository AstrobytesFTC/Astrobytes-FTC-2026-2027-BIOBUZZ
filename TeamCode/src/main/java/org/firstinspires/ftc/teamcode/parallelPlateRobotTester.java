package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class parallelPlateRobotTester extends LinearOpMode {
    DcMotorEx pollenTurret;
//    DcMotorEx nectarTurret;
    DcMotorEx feeder;
    Servo nectarServo;
    Servo pollenServo;
    CRServo flowerServo;
//    DcMotorEx intake;

    double targetVelocity = 1000;
    boolean lastUp, lastDown;
    double pollenPos = 0.5, nectarPos = 0.5;
    final double TURRET_RANGE_DEG = 360; // turret degrees covered by servo 0 to 1

    double aim(double x, double y, double current) {
        if (Math.hypot(x, y) < 0.5) return current; // stick released, hold position
        return Range.clip(0.5 + Math.toDegrees(Math.atan2(x, -y)) / TURRET_RANGE_DEG, 0, 1);
    }
    @Override
    public void runOpMode() throws InterruptedException {

        pollenTurret = hardwareMap.get(DcMotorEx.class, "pt");
//        nectarTurret = hardwareMap.get(DcMotorEx.class, "nt");
        nectarServo = hardwareMap.get(Servo.class, "ns");
        pollenServo = hardwareMap.get(Servo.class, "pss");
        flowerServo = hardwareMap.get(CRServo.class, "fs");
//        intake = hardwareMap.get(DcMotorEx.class, "intake");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        pollenTurret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        nectarTurret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            // Adjust target velocity
            if (gamepad1.dpad_up && !lastUp) targetVelocity += 50;
            if (gamepad1.dpad_down && !lastDown) targetVelocity -= 50;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            // Test controls
            if (gamepad1.a) pollenTurret.setVelocity(targetVelocity);
//            if (gamepad1.b) nectarTurret.setVelocity(targetVelocity);
            if (gamepad1.x) pollenTurret.setVelocity(-targetVelocity);
//            if (gamepad1.y) nectarTurret.setVelocity(-targetVelocity);
            if (gamepad1.back) {
                pollenTurret.setVelocity(0);
//                nectarTurret.setVelocity(0);
            }

            // Test FEEDER Servo
            feeder.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

            //Test ROTATION

            pollenPos = aim(gamepad1.left_stick_x, gamepad1.left_stick_y, pollenPos);
            nectarPos = aim(gamepad1.right_stick_x, gamepad1.right_stick_y, nectarPos);
            pollenServo.setPosition(pollenPos);
            nectarServo.setPosition(nectarPos);

//            pollenPos = Range.clip(pollenPos + gamepad1.left_stick_x * 0.005, 0, 1);
//            nectarPos = Range.clip(nectarPos + gamepad1.right_stick_x * 0.005, 0, 1);
//            pollenServo.setPosition(pollenPos);
//            nectarServo.setPosition(nectarPos);

            //Test FLOWER Servo

            if(gamepad1.dpad_left){
                flowerServo.setPower(-1);
                sleep(500);
                flowerServo.setPower(0);
            }
            if(gamepad1.dpad_right){
                flowerServo.setPower(0);
                sleep(500);
                flowerServo.setPower(-1);
            }
            //Test INTAKE
//            intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);


            // Telemetry
            telemetry.addData("Target", targetVelocity);
            telemetry.addData("Pollen Turret velocity", pollenTurret.getVelocity());
//            telemetry.addData("Nectar Turret velocity", nectarTurret.getVelocity());
            telemetry.addData("Pollen amps", pollenTurret.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Nectar amps", nectarTurret.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Battery V", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addData("Pollen pos", pollenServo.getPosition());
            telemetry.addData("Nectar pos", nectarServo.getPosition());
            telemetry.addData("Flower pos", flowerServo.getPower());

            telemetry.update();
        }
    }
}