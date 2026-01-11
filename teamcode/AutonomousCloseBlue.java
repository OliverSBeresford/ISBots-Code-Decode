package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Autonomous(name="Autonomous Close (Blue Tag 24)")
public class AutonomousCloseBlue extends AutonomousClose {
    @Override
    public void init() {
        startHardware();
        setAprilTagID(RobotUtils.RED_TAG_ID);
        telemetry.addLine("Robot Ready.");
        telemetry.addLine("AutoAim: uses AprilTag ID 20 (blue basket).");
        telemetry.update();
    }
}