package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.openftc.apriltag.AprilTagDetection;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoAlignLimelightOpMode extends OpMode
{
    private Limelight3A limelight;
    private TurretLimelight turret = new TurretLimelight();

    @Override
    public void init
            () {
        limelight= hardwareMap.get(Limelight3A.class, "limelight");
        turret.init(hardwareMap);
        limelight.pipelineSwitch(0);
        telemetry.addLine("Initialized all mechanisms");
        telemetry.update();

    }

    @Override
    public void start
            () {
        turret.resetTimer();
        limelight.start();

    }

    @Override
    public void loop
            () {
        LLResult result = limelight.getLatestResult();
        turret.update(result);
        if (result != null && result.isValid())
        {
            telemetry.addData("Target TX", result.getTx());
            telemetry.addData("Target TY", result.getTy());
        }else
        {
            telemetry.addLine("No Tag Detected.");

        }
        telemetry.update();

    }

    @Override
    public void stop
            (){
        turret.stop();
        limelight.stop();

    }
}