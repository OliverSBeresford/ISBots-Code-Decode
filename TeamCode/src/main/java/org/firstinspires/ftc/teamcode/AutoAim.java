package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp(name="AutoAim (Blue Tag 20)")
public class AutoAim extends OpMode {

    private static final int BLUE_BASKET_TAG_ID = 20;   // change to 24 for red
    private static final double FALLBACK_RPM = 3200;    // used if tag not visible

    private RobotUtils robot = null;

    private boolean intakeWasPressed = false; // debounce for A
    private boolean yWasPressed = false;      // tap-to-shoot for Y

    @Override
    public void init() {
        robot = new RobotUtils(hardwareMap);
        robot.setAprilTagID(BLUE_BASKET_TAG_ID);
        telemetry.addLine("Robot Ready.");
        telemetry.addLine("AutoAim: uses AprilTag ID 20 (blue basket).");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ===== DRIVE / STRAFE =====
        if (gamepad1.b) {
            robot.reset_imu_yaw();
        }

        if (gamepad1.left_bumper) {
            robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            robot.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // ===== INTAKE MOTOR TOGGLE (A) =====
        if (gamepad1.a && !intakeWasPressed) {
            robot.toggle_motor();
            intakeWasPressed = true;
        } else if (!gamepad1.a) {
            intakeWasPressed = false;
        }

        // ===== SHOOT (TAP Y) =====
        boolean yNow = gamepad1.y;
        if (yNow && !yWasPressed) {
            // Start a full auto-shot sequence: align -> set RPM -> shoot
            robot.requestAutoShot(FALLBACK_RPM);
        }
        yWasPressed = yNow;

        // Stop shooter (X)
        if (gamepad1.x) {
            robot.stopShooter();
        }

        // Must be called every loop
        robot.updateShooter();

        // Get data for telemetry
        AprilTagPoseFtc pose = robot.get_apriltag_data();
        double recommendedRpm = robot.calculateRPM();
        // ===== TELEMETRY =====
        telemetry.addData("Tag Seen?", (pose != null));
        telemetry.addData("Tag ID", BLUE_BASKET_TAG_ID);

        if (pose != null) {
            telemetry.addData("Range (in)", String.format("%.1f", pose.range));
            telemetry.addData("Bearing (deg)", String.format("%.1f", pose.bearing));
            telemetry.addData("Elevation (deg)", String.format("%.1f", pose.elevation));
        } else {
            telemetry.addData("Range (in)", "N/A");
        }

        telemetry.addData("Recommended RPM", String.format("%.0f", recommendedRpm));
        telemetry.addData("Shooter State", robot.launchState);
        telemetry.addData("Launcher vel (rad)", robot.leftLaunch.getVelocity(AngleUnit.RADIANS));

        telemetry.update();
    }
}