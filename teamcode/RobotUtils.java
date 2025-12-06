package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotUtils {
    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor rightDrive = null;
    public IMU imu = null;
    public CRServo intake = null;
    public DcMotorEx leftLaunch = null;
    public DcMotorEx rightLaunch = null;
    
    public RobotUtils(HardwareMap hardwareMap) {
        // Initialize all hardware components based on the provided parameters
        // This is a placeholder for actual initialization code
        this.frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        this.frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        this.backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        this.backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        this.imu = hardwareMap.get(IMU.class, "imu");
        this.intake = hardwareMap.get(CRServo.class, "servo_picky_uppy");
        this.leftLaunch = hardwareMap.get(DcMotorEx.class, "left_launch");
        this.rightLaunch = hardwareMap.get(DcMotorEx.class, "right_launch");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Make sure the robot breaks when you let go of the controller
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Matches the orientation of our robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    // This function drives the robot field-relative
    public void driveFieldRelative(double forward, double right, double rotate) {
        if (imu == null) {
            // If there is no IMU, fall back to robot-relative driving
            drive(forward, right, rotate);
            return;
        }

        // If any of the drive motors are not initialized, do nothing
        else if (backLeftDrive == null || backRightDrive == null || frontLeftDrive == null || frontRightDrive == null) return;

        // Convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        // If any of the drive motors are not initialized, do nothing
        if (backLeftDrive == null || backRightDrive == null || frontLeftDrive == null || frontRightDrive == null) return;

        // Calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;


        // Normalize the wheel powers if any exceeds 1.0
        double powers[] = {frontLeftPower, frontRightPower, backRightPower, backLeftPower};
        if (max_magnitude(powers) > 1.0) {
            // Normalize the powers so no wheel power exceeds 1.0
            double maxPower = max_magnitude(powers);
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
            backLeftPower /= maxPower;
        }


        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    public double max_magnitude(double... numbers) {
        double maximum = 0;

        for (double number : numbers) {
            maximum = Math.max(Math.abs(number), maximum);
        }

        return maximum;
    }

    public void toggle_servo() {
        if (intake == null) return;

        intake.setDirection(CRServo.Direction.REVERSE);

        if (intake.getPower() == 0) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }
}
