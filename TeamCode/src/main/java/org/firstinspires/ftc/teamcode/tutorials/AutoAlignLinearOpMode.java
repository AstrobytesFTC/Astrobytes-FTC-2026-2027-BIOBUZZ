package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class AutoAlignLinearOpMode extends LinearOpMode {
    private Limelight3A limelight;
    private TurretLimelightNew pollenTurret  = new TurretLimelightNew();
    private TurretLimelightNew nectarTurret  = new TurretLimelightNew();


    @Override
    public void runOpMode() {
        // init
        limelight= hardwareMap.get(Limelight3A.class, "limelight");
        pollenTurret.init(hardwareMap, "pss");
        nectarTurret.init(hardwareMap, "ns");

        limelight.pipelineSwitch(0);
        telemetry.addLine("Initialized all mechanisms");
        telemetry.update();

        waitForStart();

        // start
        pollenTurret.resetTimer();
        nectarTurret.resetTimer();

        limelight.start();

        // loop
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            pollenTurret.update(result, telemetry, "Pollen");
            nectarTurret.update(result, telemetry, "Nectar");

            if (result != null && result.isValid()) {
                telemetry.addData("Target TX", result.getTx());
                telemetry.addData("Target TY", result.getTy());
            }else {
                telemetry.addLine("No Tag Detected.");

            }
            telemetry.update();

        }

        // stop
        pollenTurret.stop();
        nectarTurret.stop();

        limelight.stop();

    }
}