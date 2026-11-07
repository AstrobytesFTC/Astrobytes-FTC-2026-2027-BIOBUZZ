package org.firstinspires.ftc.teamcode.tutorials;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class TurretLimelightNew {
    private Servo turretServo;
    private double kP = 0.001;
    private double kD = 0;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.5;
    private final double MAX_STEP = 0.01;

    private double minPosition = 0.3;
    private double maxPosition = 0.8;
    private double currentPosition = 0.5;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();
    public void init(HardwareMap hwMap, String servoName) {

        turretServo = hwMap.get(Servo.class, servoName);
        turretServo.setPosition(currentPosition);



    }

    //kP
    public void setkD(double newkD){
        kD = newkD;

    }
    public double getkD(){
        return kD;

    }

    //kP
    public void setkP(double newkP){
        kP = newkP;

    }
    public double getkP(){
        return kP;

    }
    public void resetTimer(){
        timer.reset();
        lastError = 0;


    }
    // () part might need a change below
    public void update(LLResult result, Telemetry telemetry, String name){
        double deltaTime = timer.seconds();
        timer.reset();
        if (result == null || !result.isValid()) {
//            turret.setPower(0);
            lastError = 0;
            timer.reset();
            return;

        }
// start pD controlller
        //if turret moves away from target put result.getTx first
        double error = goalX - result.getTx();
        double pTerm = error * kP;
        double dTerm = 0;

        if (deltaTime > 0) {
            dTerm = ((error-lastError)/ deltaTime) * kD;
            lastError = error;

        }
        if (Math.abs(error)< angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_STEP, MAX_STEP);

        }

//safety encoder check or magentic limit switch check or other safeties

        double newPosition = Range.clip(currentPosition + power, minPosition, maxPosition);

        currentPosition = newPosition;
        turretServo.setPosition(currentPosition);
        telemetry.addData(name + " Error", error);
        telemetry.addData(name + " P Term", pTerm);
        telemetry.addData(name +" D Term", dTerm);
        telemetry.addData(name +" Power", power);
        telemetry.addData(name + " Servo Position", currentPosition);
        telemetry.addData(name +" Delta Time", deltaTime);
    }
    public void stop (){


    }
}