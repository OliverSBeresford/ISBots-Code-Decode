package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TwoPlayers Blue")
public class TwoPlayersBlue extends TwoPlayersBoth {
    @Override
    public void init() {
        startHardware();
        setAprilTagID(RobotUtils.BLUE_TAG_ID);
        telemetry.addLine("Robot Ready.");
        telemetry.addLine("Driver = Gamepad1 (drive)");
        telemetry.addLine("Operator = Gamepad2 (intake + shooter)");
        telemetry.update();
    }
}