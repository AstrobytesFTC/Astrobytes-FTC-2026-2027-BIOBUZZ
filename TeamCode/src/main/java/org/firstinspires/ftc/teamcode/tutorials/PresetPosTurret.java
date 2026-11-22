 package org.firstinspires.ftc.teamcode.tutorials;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.Servo;


 @TeleOp
 public class PresetPosTurret extends OpMode {

     private Servo pollenTurret;
     private Servo nectarTurret;

     private final double[] pollenPresets = {0.30, 0.45, 0.60, 0.80};
     private final double[] nectarPresets = {0.30, 0.45, 0.60, 0.80};
     private int index = -1;
     private boolean lastB = false;

     @Override
     public void init() {
         pollenTurret = hardwareMap.get(Servo.class, "pss");
         nectarTurret = hardwareMap.get(Servo.class, "ns");
         telemetry.addLine("Initialized");
         telemetry.update();
     }

     @Override
     public void loop() {
         if (gamepad2.b && !lastB) {
             index = (index + 1) % 4;
             pollenTurret.setPosition(pollenPresets[index]);
             nectarTurret.setPosition(nectarPresets[index]);

         }
         lastB = gamepad2.b;

         telemetry.addData("Preset #", index + 1);
         telemetry.update();


     }
 }