package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private enum LaunchState {
        OFF,
        SPINNING_UP,
        READY,
        FEEDING
    }

    // Constants
    private static final double TOLERANCE = 0.1; // Tolerance for velocity checks in RPM
    private final int MAX_VELOCITY = 6000; // Max velocity of the Yellow Jacket motors in RPM

    private static final double FEED_DELAY = 0.5; // seconds to wait after Y before feeding
    private double readyTime = 0.0;               // time when we entered READY with a feed request

    // Variables to keep track of launch state
    public LaunchState launchState = LaunchState.OFF;
    private double targetVelocity = 0.0;
    private boolean feedRequested = false;
    private double feedingDuration = 2; // seconds
    private double feedingStartTime = 0.0;
    private double feedingStopTime = 0.0;

    // Hardware components
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor rightDrive = null;
    private IMU imu = null;
    private DcMotor intake = null;
    private CRServo feed = null;
    public DcMotorEx leftLaunch = null;
    private DcMotorEx rightLaunch = null;
    
    public RobotUtils(HardwareMap hardwareMap) {
        // Initialize all hardware components based on the provided parameters
        this.frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        this.frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        this.backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        this.backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        this.imu = hardwareMap.get(IMU.class, "imu");
        this.intake = hardwareMap.get(DcMotorEx.class, "motor_picky_uppy");
        this.leftLaunch = hardwareMap.get(DcMotorEx.class, "left_launch");
        this.rightLaunch = hardwareMap.get(DcMotorEx.class, "right_launch");
        this.feed = hardwareMap.get(CRServo.class, "servo_feeder");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightLaunch.setDirection(DcMotor.Direction.REVERSE);

        // Drivetrain motors run without encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Launch motors run using encoders to be able to control their speed
        leftLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Make sure the robot breaks when you let go of the controller
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void toggle_motor() {
        if (intake == null) return;

        intake.setDirection(DcMotor.Direction.REVERSE);

        if (intake.getPower() == 0) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0.0);
        }
    }

    public void reset_imu_yaw() {
        if (imu == null) return;

        imu.resetYaw();
    }

    public void set_launch_power(double power) {
        if (leftLaunch == null || rightLaunch == null) return;

        leftLaunch.setVelocity(power * MAX_VELOCITY * 2 * Math.PI / 60, AngleUnit.RADIANS);
        rightLaunch.setVelocity(power * MAX_VELOCITY * 2 * Math.PI / 60, AngleUnit.RADIANS);
    }

    public void set_launch_velocity(double velocity) {
        if (leftLaunch == null || rightLaunch == null) return;

        leftLaunch.setVelocity(velocity, AngleUnit.RADIANS);
        rightLaunch.setVelocity(velocity, AngleUnit.RADIANS);
    }

    public void feed_to_launch(double power) {
        if (feed == null) return;

        feed.setPower(-power);
    }

    public void startShooter(double velocityRadPerSec) {
        targetVelocity = velocityRadPerSec;
        set_launch_velocity(velocityRadPerSec);
        launchState = LaunchState.SPINNING_UP;
    }

    public void stopShooter() {
        set_launch_velocity(0);
        feed_to_launch(0);
        launchState = LaunchState.OFF;
    }

    public void shootBallWhenReady() {
        feedRequested = true;
    }

    public void updateShooter() {
        double vel = leftLaunch.getVelocity(AngleUnit.RADIANS);

        switch (launchState) {

            case SPINNING_UP:
                if (Math.abs(vel - targetVelocity) < TOLERANCE) {
                    launchState = LaunchState.READY;
                }
                break;

            case READY:
                if (feedRequested) {
                    // remember the time we got the request
                    readyTime = System.currentTimeMillis() / 1000.0;

                    // move to FEEDING state, but we will WAIT 2 seconds before actually feeding
                    launchState = LaunchState.FEEDING;

                    // reset request so it doesn't retrigger every loop
                    feedRequested = false;

                    // reset feeding timer markers so FEEDING can start fresh
                    feedingStartTime = 0.0;
                    feedingStopTime = 0.0;
                }
                break;

            case FEEDING:
                double currentTime = System.currentTimeMillis() / 1000.0;

                // Wait 2 seconds after READY before starting the feeder
                if (feedingStartTime == 0.0 && currentTime >= readyTime + FEED_DELAY) {
                    feed_to_launch(1.0);  // start feeder
                    feedingStartTime = currentTime;
                    feedingStopTime = feedingStartTime + feedingDuration;
                }

                // Once feeder has started, stop it after feedingDuration seconds
                if (feedingStartTime != 0.0 && currentTime >= feedingStopTime) {
                    feed_to_launch(0.0);  // stop feeder
                    launchState = LaunchState.OFF;
                }
                break;

            case OFF:
                if (vel > TOLERANCE) {
                    set_launch_velocity(0.0);
                }
                break;

            default:
                // do nothing
                break;
        }
    }

    private boolean approximately_equal(double a, double b, double tolerance) {
        return Math.abs(a - b) <= tolerance;
    }
}
