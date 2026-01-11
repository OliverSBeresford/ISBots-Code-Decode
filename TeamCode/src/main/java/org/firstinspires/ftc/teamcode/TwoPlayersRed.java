package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TwoPlayers Red")
public class TwoPlayersRed extends TwoPlayersBoth {
    @Override
    public void init() {
        robot = new RobotUtils(hardwareMap);
        robot.setAprilTagID(RobotUtils.RED_TAG_ID);
        telemetry.addLine("Robot Ready.");
        telemetry.addLine("Driver = Gamepad1 (drive)");
        telemetry.addLine("Operator = Gamepad2 (intake + shooter)");
        telemetry.update();
    }
}