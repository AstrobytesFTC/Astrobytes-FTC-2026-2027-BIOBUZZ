package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class AutoAlignOpmodeNew extends OpMode {
    private Limelight3A limelight;
    private TurretLimelightNew pollenTurret  = new TurretLimelightNew();
    private TurretLimelightNew nectarTurret  = new TurretLimelightNew();


    @Override
    public void init() {
        limelight= hardwareMap.get(Limelight3A.class, "limelight");
        pollenTurret.init(hardwareMap, "pss");
        nectarTurret.init(hardwareMap, "ns");

        limelight.pipelineSwitch(0);
        telemetry.addLine("Initialized all mechanisms");
        telemetry.update();

    }

    @Override
    public void start() {
        pollenTurret.resetTimer();
        nectarTurret.resetTimer();

        limelight.start();

    }

    @Override
    public void loop() {
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

    @Override
    public void stop(){
        pollenTurret.stop();
        nectarTurret.stop();

        limelight.stop();

    }
}