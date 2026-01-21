package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;

import android.util.Size;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

abstract public class RobotUtils extends OpMode {
    public enum LaunchState {
        OFF,
        SPINNING_UP,
        READY,
        FEEDING,
        REVERSING
    }

    public enum DriveState {
        TURNING,
        DRIVING_TIME,
        ALIGNING,
        STOPPED
    }

    // Constants
    private static final double TOLERANCE = 0.1; // Tolerance for velocity checks in RPM

    // Variables to keep track of launch state
    public LaunchState launchState = LaunchState.OFF;
    private double targetVelocity = 0.0;
    private boolean feedRequested = false;
    private final double FEEDING_DURATION = 2; // seconds
    public double feedingStartTime = 0.0;
    public double reverseStartTime = 0.0;
    // Auto-shot (AprilTag align -> spin -> feed) variables
    private double autoShotRpm = 0.0;
    private boolean autoShotRequested = false;
    private double FALLBACK_RPM = 2400.0;
    private double REVERSE_DURATION = 0.6;

    // Drive state
    public DriveState driveState = DriveState.STOPPED;
    private double targetYaw = 0.0;
    private double driveStartTime = 0.0;
    private double driveEndTime = 0.0;
    private double imuOffset = 0.0;
    private double driveTimePower = 0.0;

    // Hardware components
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private IMU imu = null;
    private DcMotor intake = null;
    private CRServo feed = null;
    public DcMotorEx leftLaunch = null;
    private DcMotorEx rightLaunch = null;

    // Vision variables
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static final int BLUE_TAG_ID = 20;
    public static final int RED_TAG_ID = 24;
    int tagID = 20; // default tag ID for alignment

    protected void startHardware() {
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
        initAprilTag(hardwareMap);
    }

    public void setAprilTagID(int id) {
        tagID = id;
    }

    // This function drives the robot field-relative
    public void driveFieldRelative(double forward, double right, double rotate) {
        if (imu == null) {
            // If there is no IMU, fall back to robot-relative driving
            drive(forward, right, rotate);
            return;
        }

        // If any of the drive motors are not initialized, do nothing
        else if (backLeftDrive == null || backRightDrive == null || frontLeftDrive == null || frontRightDrive == null)
            return;

        // Convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + imuOffset));

        // Convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        // If any of the drive motors are not initialized, do nothing
        if (backLeftDrive == null || backRightDrive == null || frontLeftDrive == null || frontRightDrive == null)
            return;

        // Calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;


        // Normalize the wheel powers if any exceeds 1.0
        double[] powers = {frontLeftPower, frontRightPower, backRightPower, backLeftPower};
        if (maxMagnitude(powers) > 1.0) {
            // Normalize the powers so no wheel power exceeds 1.0
            double maxPower = maxMagnitude(powers);
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

    public void driveForSeconds(double seconds, double power) {
        driveStartTime = System.currentTimeMillis() / 1000.0;
        driveEndTime = driveStartTime + seconds;
        driveTimePower = power;
        driveState = DriveState.DRIVING_TIME;
    }

    public void turnToAngle(double targetAngle) {
        double kP = 0.01;

        double error = targetAngle - imu.getRobotYawPitchRollAngles().getYaw();
        double turn = error * kP;

        turn = clamp(turn, -0.4, 0.4);
        drive(0, 0, turn);

        if (Math.abs(error) < 3.0 && driveState == DriveState.TURNING)
            driveState = DriveState.STOPPED;
    }

    public void turnDegrees(double degrees) {
        targetYaw = imu.getRobotYawPitchRollAngles().getYaw() + degrees;

        driveState = DriveState.TURNING;
    }

    public void turnToGoal() {
        // Turn to face one the goals, starting facing toward the wall with the artifact info
        if (tagID == BLUE_TAG_ID) {
            turnDegrees(45);
        } else if (tagID == RED_TAG_ID) {
            turnDegrees(-45);
        }
    }

    public double maxMagnitude(double... numbers) {
        double maximum = 0;

        for (double number : numbers) {
            maximum = Math.max(Math.abs(number), maximum);
        }

        return maximum;
    }
    
    public void toggleFeeder() {
        if (feed == null) return;

        if (feed.getPower() == 0) {
            feed.setPower(1.0);
        } else {
            feed.setPower(0.0);
        }
    }

    public void toggleMotor() {
        if (intake == null) return;

        intake.setDirection(DcMotor.Direction.REVERSE);

        if (intake.getPower() == 0) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0.0);
        }
    }

    public void resetImuYaw() {
        if (imu == null) return;

        imu.resetYaw();
    }

    public void setLaunchPower(double power) {
        if (leftLaunch == null || rightLaunch == null) return;

        leftLaunch.setPower(power);
        rightLaunch.setPower(power);
    }

    public void setLaunchVelocity(double velocity) {
        if (leftLaunch == null || rightLaunch == null) return;

        leftLaunch.setVelocity(velocity, AngleUnit.RADIANS);
        rightLaunch.setVelocity(velocity, AngleUnit.RADIANS);
    }

    public double getLaunchVelocity() {
        if (leftLaunch == null || rightLaunch == null) return 0.0;

        return leftLaunch.getVelocity(AngleUnit.RADIANS);
    }

    public void feedToLaunch(double power) {
        if (feed == null) return;

        feed.setPower(-power);
    }

    public void startShooter(double velocityRPM) {
        targetVelocity = velocityRPM * 2.0 * Math.PI / 6000.0;
        reverseStartTime = System.currentTimeMillis() / 1000.0;
        launchState = LaunchState.REVERSING;
    }

    public void stopShooter() {
        setLaunchVelocity(0);
        feedToLaunch(0);
        launchState = LaunchState.OFF;
    }

    public void shootBallWhenReady() {
        feedRequested = true;
    }

    public void requestAutoShot() {
        // Calculate recommended RPM based on current range to tag
        autoShotRpm = calculateRPM();
        // Use the provided fallback RPM if we can't see the tag
        if (autoShotRpm == 0.0) autoShotRpm = FALLBACK_RPM;

        // Kick the state machine into aligning immediately
        driveState = DriveState.ALIGNING;

        autoShotRequested = true;
    }

    private void updateShooter() {
        double vel = leftLaunch.getVelocity(AngleUnit.RADIANS);

        switch (launchState) {
            case REVERSING:
                setLaunchPower(-0.1);
                if (System.currentTimeMillis() / 1000.0 > reverseStartTime + REVERSE_DURATION) {
                    setLaunchPower(0);
                    setLaunchVelocity(targetVelocity);
                    launchState = LaunchState.SPINNING_UP;
                }

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
                    feedToLaunch(1);

                    // reset request so it doesn't retrigger every loop
                    feedRequested = false;
                }
                break;

            case FEEDING:
                double currentTime = System.currentTimeMillis() / 1000.0;

                // Wait 2 seconds after starting feeding
                if (currentTime >= feedingStartTime + FEEDING_DURATION) {
                    feedToLaunch(0.0);  // stop feeder
                    launchState = LaunchState.OFF;
                }

                break;

            case OFF:
                if (vel > TOLERANCE) {
                    setLaunchVelocity(0.0);
                }
                break;

            default:
                // do nothing
                break;
        }
    }

    private void updateDrive() {
        switch (driveState) {
            case ALIGNING:
                // Override driving while aligning
                if (isAligned()) {
                    driveState = DriveState.STOPPED;

                    // stop movement once aligned
                    drive(0, 0, 0);

                    // Don't start the shooter if they didn't request to shoot
                    if (!autoShotRequested) return;

                    // Start shooter at the RPM we computed from range
                    startShooter(autoShotRpm);   // <-- sets launchState = REVERSING

                    // IMPORTANT: keep a request to feed once we reach speed
                    feedRequested = true;

                    // Note: startShooter already moved us to REVERSING state
                } else {
                    alignToAprilTag();
                }
                break;

            case TURNING:
                turnToAngle(targetYaw);
                break;

            case DRIVING_TIME:
                if (System.currentTimeMillis() / 1000.0 > driveEndTime) {
                    driveState = DriveState.STOPPED;
                    return;
                }
                drive(driveTimePower, driveTimePower, 0);
                break;

            default:
                break;
        }
    }

    public void update() {
        updateShooter();
        updateDrive();

        telemetry.addData("Shooter state", launchState);
        telemetry.addData("Drive state", driveState);
        telemetry.update();
    }

    public boolean isShotCompleted() {
        return launchState == LaunchState.OFF && getLaunchVelocity() < TOLERANCE;
    }

    public boolean isStopped() {
        return driveState == DriveState.STOPPED;
    }

    private boolean approximately_equal(double a, double b, double tolerance) {
        return Math.abs(a - b) <= tolerance;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(HardwareMap hardwareMap) {

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
                                -0.05, 0.1535, 0.0508, 0
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
    public AprilTagPoseFtc getApriltagData() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == tagID) {
                return detection.ftcPose;
            }
        }

        return null;
    }

    public void alignToAprilTag() {
        AprilTagPoseFtc pose = getApriltagData();
        if (pose == null) {
            drive(0, 0, 0); // Stop if no tag
            return;
        }

        // Errors
        double strafeError = 0;       // Ignore strafe error for now
        double bearingError = pose.bearing;      // want bearing = 0

        // Proportional control
        double right = strafeError * 0.05;
        double forward = 0;                    // no forward movement
        double rotate = -bearingError * 0.05;

        // Clamp power (very important)
        forward = clamp(forward, -0.4, 0.4);
        right = clamp(right, -0.4, 0.4);
        rotate = clamp(rotate, -0.3, 0.3);

        drive(forward, right, rotate);
    }

    public boolean isAligned() {
        AprilTagPoseFtc pose = getApriltagData();
        if (pose == null) return true;

        return Math.abs(pose.bearing) < 5.0; // only bearing for now;
    }

    public double calculateRPM() {
        if (leftLaunch == null) return 0.0;

        AprilTagPoseFtc pose = getApriltagData();
        if (pose == null) return 0.0;

        return rpmFromRange(pose.range);
    }

    /**
     * Distance->RPM mapping using linear interpolation.
     * Maps ranges: 106->2300, 119->2400, 145->2500, 150->3000
     **/
    public double rpmFromRange(double rangeIn) {
        if (rangeIn <= 106) return 2400;
        if (rangeIn <= 119) {
            return 2350;
        }
        if (rangeIn <= 145) {
            return 2450;
        }
        if (rangeIn <= 150) {
            return 2550;
        }
        return 2900;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
