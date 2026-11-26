
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hood Servo Test")
public class HoodServoTest extends LinearOpMode {

    private Servo phood;

    // Current calculated hood positions
    private static final double CLOSE_POSITION = 0.260;
    private static final double MIDDLE_POSITION = 0.180;
    private static final double FAR_POSITION = 0.140;

    private static final long MOVE_TIME = 2000;

    @Override
    public void runOpMode() {

        // Hardware initialization
        phood = hardwareMap.get(Servo.class, "phood");

        telemetry.addLine("Hood Servo Test Ready");
        telemetry.addLine("Servo: phood");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

            // FAR position
            phood.setPosition(FAR_POSITION);

            telemetry.addData("Hood Position", "FAR");
            telemetry.addData("Servo Command", FAR_POSITION);
            telemetry.update();

            sleep(MOVE_TIME);

            if (!opModeIsActive()) break;

            // CLOSE position
            phood.setPosition(CLOSE_POSITION);

            telemetry.addData("Hood Position", "CLOSE");
            telemetry.addData("Servo Command", CLOSE_POSITION);
            telemetry.update();

            sleep(MOVE_TIME);

            if (!opModeIsActive()) break;

            // MIDDLE position
            phood.setPosition(MIDDLE_POSITION);

            telemetry.addData("Hood Position", "MIDDLE");
            telemetry.addData("Servo Command", MIDDLE_POSITION);
            telemetry.update();

            sleep(MOVE_TIME);
        }
    }
}
