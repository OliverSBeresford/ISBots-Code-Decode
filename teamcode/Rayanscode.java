package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotUtils;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import java.net.InterfaceAddress;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Rayanscode")
public class Rayanscode extends OpMode {

    RobotUtils robot = null;
    boolean wasPressed = false;

    @Override
    public void init() {
        robot = new RobotUtils(hardwareMap);
        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ===== DRIVE / STRAFE =====
        telemetry.addLine("Press A to toggle motor of the picky uppy");
        telemetry.addLine("Press B to reset Yaw");
        telemetry.addLine("Hold left bumper = robot-relative driving");
        telemetry.addLine("No bumper = field-relative driving");

        if (gamepad1.a && !wasPressed) {
            robot.toggle_motor();
            wasPressed = true;
        } else if (!gamepad1.a) {
            wasPressed = false;
        }

        if (gamepad1.b) {
            robot.reset_imu_yaw();
        }

        if (gamepad1.left_bumper) {
            robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            robot.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // ===== SHOOTER =====
        // CHANGE THIS BUTTON if you don't want it to also toggle servo
        if (gamepad1.y) {   // use Y instead of A to avoid conflict
            double rpm = 600;
            double rad = rpm * 2 * Math.PI / 6000;
            robot.startShooter(rad);
            robot.shootBallWhenReady();
        }

        if (gamepad1.x) {
            robot.stopShooter();
        }

        if (gamepad1.right_stick_button) {
            robot.feed_to_launch(1);
        }

        if (gamepad1.left_stick_button) {
            robot.feed_to_launch(0);
        }

        robot.updateShooter();

        telemetry.addData("Shooter State", robot.launchState);
        telemetry.addData("Velocity", robot.leftLaunch.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Pos: ", robot.leftLaunch.getCurrentPosition());
        telemetry.update();
    }
}