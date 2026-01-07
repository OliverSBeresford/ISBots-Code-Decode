package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import apple.laf.JRSUIConstants.Size;

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

    // Variables to keep track of launch state
    public LaunchState launchState = LaunchState.OFF;
    private double targetVelocity = 0.0;
    private boolean feedRequested = false;
    private double feedingDuration = 2; // seconds
    public double feedingStartTime = 0.0;

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
    
    // Vision variables
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
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

        // Initialize the AprilTag processor and vision portal
        initAprilTag();
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
                    // move to FEEDING state
                    launchState = LaunchState.FEEDING;

                    // Set feeding start time
                    feedingStartTime = System.currentTimeMillis() / 1000.0;
                    feed_to_launch(1);

                    // reset request so it doesn't retrigger every loop
                    feedRequested = false;
                }
                break;

            case FEEDING:
                double currentTime = System.currentTimeMillis() / 1000.0;

                // Wait 2 seconds after starting feeding
                if (currentTime >= feedingStartTime + feedingDuration) {
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

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
            // == CAMERA CALIBRATION ==
            .setLensIntrinsics(1432.032, 1432.032, 997.085, 1432.032)
            // ... these parameters are fx, fy, cx, cy.
            .setCameraPose(
                new Position(
                    DistanceUnit.METER,
                    -1, -0.075, 0.1655, 0
                ),
                new YawPitchRollAngles(
                    AngleUnit.DEGREES,
                    0, -90, 0, 0
                )
            )

            .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    // Add data about AprilTag detections.
    private AprilTagPoseFtc get_apriltag_data(int apriltag_id) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == apriltag_id) {
                return detection.ftcPose;
            }
        }
    }
}
