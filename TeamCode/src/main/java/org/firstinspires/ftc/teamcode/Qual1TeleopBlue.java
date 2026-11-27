package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Qual1 Teleop BLUE", group = "Qual")
public class Qual1TeleopBlue extends Qual1Base {
    @Override
    protected int getMinTagId() { return 38; }

    @Override
    protected int getMaxTagId() { return 45; }
}