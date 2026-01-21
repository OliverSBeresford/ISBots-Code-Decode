package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous (Red Tag 24)")
public class AutonomousCloseRed extends AutonomousClose {
    @Override
    public void init() {
        startHardware();
        setAprilTagID(RobotUtils.RED_TAG_ID);
        telemetry.addLine("Robot Ready.");
        telemetry.addLine("AutoAim: uses AprilTag ID 20 (blue basket).");
        telemetry.update();
    }
}