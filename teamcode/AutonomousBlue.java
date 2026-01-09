package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Autonomous(name="Autonomous (Blue Tag 20)")
public class AutonomousBlue extends OpMode {

    private static final int BLUE_BASKET_TAG_ID = 20;   // change to 24 for red
    private static final double FALLBACK_RPM = 3200;    // used if tag not visible

    private RobotUtils robot = null;

    @Override
    public void init() {
        robot = new RobotUtils(hardwareMap);
        telemetry.addLine("Robot Ready.");
        telemetry.addLine("AutoAim: uses AprilTag ID 20 (blue basket).");
        telemetry.update();
    }

    @Override
    public void loop() {
    }
 // ===== SHOOT (TAP Y) =====
        boolean yNow = gamepad1.y;
        if (yNow && !yWasPressed) {
            robot.shootBallWhenReady(); // your RobotUtils will feed when ready for 2 seconds
        }
        yWasPressed = yNow;
        telemetry.addLine("Robot is shooting the 1st ball");
        telemetry.update();

}