package org.firstinspires.ftc.teamcode.tutorials;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
public class TurretLimelight
{
    private DcMotorEx turret;
    private double kP = 0.0001;
    private double kD = 0;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.5;
    private final double MAX_POWER = 0.4;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();
    public void init(HardwareMap hwMap
    ) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //kP
    public void setkD(double newkD
    ){
        kD = newkD;

    }
    public double getkD
            (){
        return kD;

    }

    //kD
    public void setkP(double newkP
    ){
        kP = newkP;

    }
    public double getkP
            (){
        return kP;

    }
    public void resetTimer
            (){
        timer.reset();

    }
    // () part might need a change below
    public void update(LLResult result
    ){
        double deltaTime = timer.seconds();
        timer.reset();
        if (result == null || !result.isValid())
        {
            turret.setPower(0);
            lastError = 0;
            return;

        }
// start pD controlller
        //if turret moves away from target put result.getTx first
        double error = goalX - result.getTx();
        double pTerm = error * kP;
        double dTerm = 0;
        if (deltaTime > 0)
        {
            dTerm = ((error-lastError)/ deltaTime) * kD;

        }
        if (Math.abs(error)< angleTolerance)
        {
            power = 0;
        } else
        {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);

        }

//safety encoder check or magentic limit switch check or other safeties
        turret.setPower(power);
        lastError = error;

    }
    public void stop
            (){
        turret.setPower(0);

    }
}