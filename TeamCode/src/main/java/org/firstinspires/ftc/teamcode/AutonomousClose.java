package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

abstract public class AutonomousClose extends RobotUtils {
    protected enum State {
        BACK_UP,
        SHOOT_BALL,
        INTAKE_BALL,
        DONE
    }

    protected double intakeStartTime = 0.0;
    protected final double INTAKE_DURATION = 2.0; // seconds
    protected State currentState = State.BACK_UP;
    protected int currentStep = 0;
    protected final State[] steps = {State.BACK_UP, State.SHOOT_BALL, State.INTAKE_BALL, State.SHOOT_BALL, State.INTAKE_BALL, State.SHOOT_BALL, State.INTAKE_BALL, State.SHOOT_BALL, State.DONE};

    private void nextStep() {
        // Update the step number
        currentStep++;
        currentState = steps[currentStep];
    }

    @Override
    public void loop() {
        switch (currentState) {
            case BACK_UP:
                // Just drive for 2.5 seconds, we don't have encoders
                // driveForSeconds(0.8, -0.5)
                driveForSeconds(0.5, -0.8);
                nextStep();
                break;

            case SHOOT_BALL:
                if (isStopped() && intakeStartTime + INTAKE_DURATION < getRuntime()) { // Aim at the blue basket tag
                    requestAutoShot();
                    nextStep();
                }
                break;

            case INTAKE_BALL:
                if (isShotCompleted()) {
                    // Records the time you start intaking the ball
                    intakeStartTime = getRuntime();

                    // Start the intake wheel
                    toggleMotor();
                    nextStep();
                    telemetry.addLine("Robot is intaking the ball");
                }
                break;

            case DONE:
                telemetry.addLine("Autonomous complete.");

                break;
        }

        // Update the robot
        update();
        telemetry.addData("Shooter State", launchState);
        telemetry.addData("Drive State", driveState);
        telemetry.addData("State", currentState);
        telemetry.addData("Start, Current Time, Over", String.format("%f, %f, %b", reverseStartTime, System.currentTimeMillis() / 1000.0, System.currentTimeMillis() / 1000.0 > reverseStartTime + 0.1));
        telemetry.addData("Launcher vel (rad)", leftLaunch.getVelocity(AngleUnit.RADIANS));

        // Get data for telemetry
        AprilTagPoseFtc pose = getApriltagData();
        double recommendedRpm = calculateRPM();
        // ===== TELEMETRY =====
        telemetry.addData("Tag Seen?", (pose != null));
        telemetry.addData("Tag ID", tagID);

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
