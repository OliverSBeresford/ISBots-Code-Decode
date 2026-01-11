package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public abstract class TwoPlayersBoth extends RobotUtils {
    // gamepad2 A toggle debounce
    protected boolean intakeWasPressed = false;

    @Override
    public void loop() {

        // =========================
        // DRIVER (GAMEPAD 1) - DRIVE
        // =========================
        telemetry.addLine("DRIVER (GP1): Left stick = drive/strafe, Right stick X = turn");
        telemetry.addLine("DRIVER (GP1): Hold LB = robot-relative, release LB = field-relative");
        telemetry.addLine("DRIVER (GP1): B = reset yaw");
        telemetry.addLine("SHOOTER (GP2): Y = auto shot");
        telemetry.addLine("SHOOTER (GP2): Right trigger = far shot");
        telemetry.addLine("SHOOTER (GP2): Left trigger = near shot");

        if (gamepad1.b) {
            resetImuYaw();
        }

        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // ==========================================
        // OPERATOR (GAMEPAD 2) - INTAKE + SHOOTER
        // ==========================================
        telemetry.addLine("SHOOTER (GP2): A = toggle intake motor");
        telemetry.addLine("SHOOTER (GP2): Y = spin up + request shot");
        telemetry.addLine("SHOOTER (GP2): X = stop shooter");

        // Toggle intake motor on GP2 A (debounced)
        if (gamepad2.a && !intakeWasPressed) {
            toggleMotor();
            intakeWasPressed = true;
        } else if (!gamepad2.a) {
            intakeWasPressed = false;
        }

        // Shooter control on GP2
        if (gamepad2.y) {
            // NOTE: This is still 23 RPM (very slow). Increase if you want more speed
            requestAutoShot();
            shootBallWhenReady();
        }

        if (gamepad2.right_trigger > 0) {
            startShooter(2525);
            shootBallWhenReady();
        }

        if (gamepad2.left_trigger > 0) {
            startShooter(2300);
            shootBallWhenReady();
        }

        if (gamepad2.x) {
            stopShooter();
        }

        // Must be called every loop
        update();

        // =========================
        // TELEMETRY
        // =========================
        telemetry.addData("Shooter State", launchState);

        double velRad = leftLaunch.getVelocity(AngleUnit.RADIANS);
        double velRpm = velRad * 6000.0 / (2.0 * Math.PI);

        telemetry.addData("Launcher Velocity (rad/s)", velRad);
        telemetry.addData("Launcher Velocity (RPM)", velRpm);

        // Get data for telemetry
        AprilTagPoseFtc pose = getApriltagData();
        double recommendedRpm = calculateRPM();
        // ===== TELEMETRY =====
        telemetry.addData("Tag Seen?", (pose != null));
        telemetry.addData("Tag ID", tagID);

        if (pose != null) {
            telemetry.addData("Range (in)", String.format("%.1f", pose.range));
        } else {
            telemetry.addData("Range (in)", "N/A");
        }

        telemetry.addData("Recommended RPM", String.format("%.0f", recommendedRpm));
        telemetry.update();

        telemetry.update();
    }
}