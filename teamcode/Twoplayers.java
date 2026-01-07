package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Rayanscode")
public class Rayanscode extends OpMode {

    private RobotUtils robot = null;

    // gamepad2 A toggle debounce
    private boolean intakeWasPressed = false;

    @Override
    public void init() {
        robot = new RobotUtils(hardwareMap);
        telemetry.addLine("Robot Ready.");
        telemetry.addLine("Driver = Gamepad1 (drive)");
        telemetry.addLine("Operator = Gamepad2 (intake + shooter)");
        telemetry.update();
    }

    @Override
    public void loop() {

        // =========================
        // DRIVER (GAMEPAD 1) - DRIVE
        // =========================
        telemetry.addLine("DRIVER (GP1): Left stick = drive/strafe, Right stick X = turn");
        telemetry.addLine("DRIVER (GP1): Hold LB = robot-relative, release LB = field-relative");
        telemetry.addLine("DRIVER (GP1): B = reset yaw");

        if (gamepad1.b) {
            robot.reset_imu_yaw();
        }

        if (gamepad1.left_bumper) {
            robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            robot.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // ==========================================
        // OPERATOR (GAMEPAD 2) - INTAKE + SHOOTER
        // ==========================================
        telemetry.addLine("OPERATOR (GP2): A = toggle intake motor");
        telemetry.addLine("OPERATOR (GP2): Y = spin up + request shot");
        telemetry.addLine("OPERATOR (GP2): X = stop shooter");

        // Toggle intake motor on GP2 A (debounced)
        if (gamepad2.a && !intakeWasPressed) {
            robot.toggle_motor();
            intakeWasPressed = true;
        } else if (!gamepad2.a) {
            intakeWasPressed = false;
        }

        // Shooter control on GP2
        if (gamepad2.y) {
            // NOTE: This is still 23 RPM (very slow). Increase if you want more speed.
            double rpm = 23;
            double radPerSec = rpm * 2 * Math.PI / 60.0;
            robot.startShooter(radPerSec);
            robot.shootBallWhenReady();
        }

        if (gamepad2.x) {
            robot.stopShooter();
        }

        // Must be called every loop
        robot.updateShooter();

        // =========================
        // TELEMETRY
        // =========================
        telemetry.addData("Shooter State", robot.launchState);

        double velRad = robot.leftLaunch.getVelocity(AngleUnit.RADIANS);
        double velRpm = velRad * 60.0 / (2.0 * Math.PI);

        telemetry.addData("Launcher Velocity (rad/s)", velRad);
        telemetry.addData("Launcher Velocity (RPM)", velRpm);

        telemetry.update();
    }
}