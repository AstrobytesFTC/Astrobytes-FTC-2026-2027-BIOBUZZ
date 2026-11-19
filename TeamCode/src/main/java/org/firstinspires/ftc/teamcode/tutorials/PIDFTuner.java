package org.firstinspires.ftc.teamcode.tutorials;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

/**
 * Velocity PIDF tuner for the pollen turret ("pt") and nectar turret ("nt").
 *
 * Open FTC Dashboard at http://192.168.43.1:8080/dash and edit values under
 * "PIDFTuner". Graph pollenTarget/pollenActual and nectarTarget/nectarActual.
 *
 * Gamepad 2 (same as your main TeleOp):
 *   Hold A       run pollen turret at target
 *   Hold B       run nectar turret at target
 *   Dpad up/down raise/lower target by STEP
 *   X            set target to 0
 *
 * You can also run either turret from Dashboard with POLLEN_ENABLED / NECTAR_ENABLED.
 *
 * Tuning order: P, I, D = 0, raise F until actual is close to target
 * (rough start: F = 32767 / max ticks per second), then raise P until it
 * reaches target without oscillating, then add D only if it overshoots.
 */
@Config
@TeleOp(name = "PIDF Tuner", group = "Tuning")
public class PIDFTuner extends LinearOpMode {

    // Shared target
    public static double TARGET_VELOCITY = 2000; // ticks per second
    public static double STEP = 50;
    public static double MIN_VELOCITY = 0;
    public static double MAX_VELOCITY = 2800;

    // Pollen turret
    public static double POLLEN_P = 10.0;
    public static double POLLEN_I = 3.0;
    public static double POLLEN_D = 0.0;
    public static double POLLEN_F = 13.0;
    public static boolean POLLEN_ENABLED = false;

    // Nectar turret
    public static double NECTAR_P = 10.0;
    public static double NECTAR_I = 3.0;
    public static double NECTAR_D = 0.0;
    public static double NECTAR_F = 13.0;
    public static boolean NECTAR_ENABLED = false;

    private DcMotorEx pollen, nectar;
    private  DcMotorEx feeder;

    private double[] lastPollen = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
    private double[] lastNectar = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    private boolean prevUp, prevDown, prevX;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pollen = hardwareMap.get(DcMotorEx.class, "pt");
        nectar = hardwareMap.get(DcMotorEx.class, "nt");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        // Match main TeleOp: pollen is reversed, nectar is not
        pollen.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{pollen, nectar}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Ready. Edit values in FTC Dashboard.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleTarget();

            lastPollen = applyIfChanged(pollen, lastPollen,
                    POLLEN_P, POLLEN_I, POLLEN_D, POLLEN_F);
            lastNectar = applyIfChanged(nectar, lastNectar,
                    NECTAR_P, NECTAR_I, NECTAR_D, NECTAR_F);

            boolean runPollen = gamepad2.a || POLLEN_ENABLED;
            boolean runNectar = gamepad2.b || NECTAR_ENABLED;

            double pollenTarget = runPollen ? TARGET_VELOCITY : 0;
            double nectarTarget = runNectar ? TARGET_VELOCITY : 0;

            pollen.setVelocity(pollenTarget);
            nectar.setVelocity(nectarTarget);

            if(gamepad2.left_bumper) feeder.setPower(1);
            if(gamepad2.right_bumper) feeder.setPower(-1);
            if(gamepad2.back) feeder.setPower(0);

            telemetry.addData("setpoint", TARGET_VELOCITY);
            telemetry.addData("pollenTarget", pollenTarget);
            telemetry.addData("pollenActual", pollen.getVelocity());
            telemetry.addData("pollenError", pollenTarget - pollen.getVelocity());
            telemetry.addData("nectarTarget", nectarTarget);
            telemetry.addData("nectarActual", nectar.getVelocity());
            telemetry.addData("nectarError", nectarTarget - nectar.getVelocity());
            telemetry.update();
        }

        pollen.setVelocity(0);
        nectar.setVelocity(0);
    }

    private void handleTarget() {
        boolean up = gamepad2.dpad_up, down = gamepad2.dpad_down, x = gamepad2.x;

        if (up && !prevUp) TARGET_VELOCITY += STEP;
        if (down && !prevDown) TARGET_VELOCITY -= STEP;
        if (x && !prevX) TARGET_VELOCITY = 0;
        TARGET_VELOCITY = Range.clip(TARGET_VELOCITY, MIN_VELOCITY, MAX_VELOCITY);

        prevUp = up;
        prevDown = down;
        prevX = x;
    }

    // Only talks to the hub when a value actually changed
    private double[] applyIfChanged(DcMotorEx motor, double[] last,
                                    double p, double i, double d, double f) {
        if (p != last[0] || i != last[1] || d != last[2] || f != last[3]) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(p, i, d, f));
            return new double[]{p, i, d, f};
        }
        return last;
    }
}