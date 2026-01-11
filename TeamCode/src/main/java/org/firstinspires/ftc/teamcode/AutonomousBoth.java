package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

abstract public class AutonomousBoth extends OpMode {
    protected enum State {
        SHOOT_FIRST_BALL,
        INTAKE_BALL,
        SHOOT_SECOND_BALL,
        ADVANCE_TO_SHOOTING_ZONE,
        TURN_BACK,
        DONE
    }

    protected double intakeStartTime = 0.0;
    protected final double INTAKE_DURATION = 2.0; // seconds
    protected RobotUtils robot = null;

    protected State currentState = State.SHOOT_FIRST_BALL;

    @Override
    public void loop() {
        switch (currentState) {
            case ADVANCE_TO_SHOOTING_ZONE:
                if (robot.isShotCompleted()) {
                    // Just drive for 2.5 seconds, we don't have encoders
                    robot.driveForSeconds(2.5, 0.4);
                    currentState = State.DONE;
                    break;
                }

            case TURN_BACK:
                if (robot.isStopped()) {
                    robot.turnDegrees(10);
                    currentState = State.DONE;
                }
                break;

            case SHOOT_FIRST_BALL:
                if (robot.isStopped()) { // Aim at the blue basket tag
                    robot.requestAutoShot();
                    currentState = State.INTAKE_BALL;
                }
                break;

            case INTAKE_BALL:
                if (robot.isShotCompleted()) {
                    // Records the time you start intaking the ball
                    intakeStartTime = getRuntime();

                    // Start the intake wheel
                    robot.toggleMotor();
                    telemetry.addLine("Robot is intaking the ball");

                    // Update robot state
                    currentState = State.SHOOT_SECOND_BALL;
                }
                break;

            case SHOOT_SECOND_BALL:
                if (getRuntime() - intakeStartTime > INTAKE_DURATION) {
                    robot.requestAutoShot();
                    robot.toggleMotor();
                    currentState = State.ADVANCE_TO_SHOOTING_ZONE;
                }
                break;

            case DONE:
                telemetry.addLine("Autonomous complete.");
                break;
        }
        robot.update();
        telemetry.addData("Shooter State", robot.launchState);
        telemetry.addData("Drive State", robot.driveState);
        telemetry.addData("Start, Current Time, Over", String.format("%f, %f, %b", robot.reverseStartTime, System.currentTimeMillis() / 1000.0, System.currentTimeMillis() / 1000.0 > robot.reverseStartTime + 0.1));
        telemetry.addData("Launcher vel (rad)", robot.leftLaunch.getVelocity(AngleUnit.RADIANS));

        // Get data for telemetry
        AprilTagPoseFtc pose = robot.getApriltagData();
        double recommendedRpm = robot.calculateRPM();
        // ===== TELEMETRY =====
        telemetry.addData("Tag Seen?", (pose != null));
        telemetry.addData("Tag ID", robot.tagID);

        if (pose != null) {
            telemetry.addData("Range (in)", String.format("%.1f", pose.range));
            telemetry.addData("Bearing (deg)", String.format("%.1f", pose.bearing));
            telemetry.addData("Elevation (deg)", String.format("%.1f", pose.elevation));
        } else {
            telemetry.addData("Range (in)", "N/A");
        }

        telemetry.addData("Recommended RPM", String.format("%.0f", recommendedRpm));
        telemetry.update();
    }
}
