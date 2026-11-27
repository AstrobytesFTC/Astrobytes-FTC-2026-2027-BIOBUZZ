package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Qual1 Teleop RED", group = "Qual")
public class Qual1TeleopRed extends Qual1Base {
    @Override
    protected int getMinTagId() { return 30; }

    @Override
    protected int getMaxTagId() { return 37; }
}