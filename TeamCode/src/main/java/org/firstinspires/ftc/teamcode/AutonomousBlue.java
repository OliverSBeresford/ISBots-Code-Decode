package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous (Blue Tag 20)")
public class AutonomousBlue extends OpMode {
    private enum State {
        SHOOT_FIRST_BALL,
        INTAKE_BALL,
        SHOOT_SECOND_BALL,
        DONE
    }

    private static final int BLUE_BASKET_TAG_ID = 20;   // change to 24 for red

    private double intakeStartTime = 0.0;
    private final double INTAKE_DURATION = 2.0; // seconds
    private RobotUtils robot = null;

    private State currentState = State.SHOOT_FIRST_BALL;

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
        switch (currentState) {
            case SHOOT_FIRST_BALL:
                // Aim at the blue basket tag
                robot.requestAutoShot();
                currentState = State.INTAKE_BALL;
                break;

            case INTAKE_BALL:
                if (robot.isShotCompleted()) {
                    // Records the time you start intaking the ball
                    intakeStartTime = getRuntime();

                    // Start the intake wheel
                    robot.toggleMotor();
                    telemetry.addLine("Robot is intaking the ball");

                    // Update robot state
                    currentState = State.INTAKE_BALL;
                }
                currentState = State.SHOOT_SECOND_BALL;
                break;

            case SHOOT_SECOND_BALL:
                if (getRuntime() - intakeStartTime > INTAKE_DURATION) {
                    robot.requestAutoShot();
                    currentState = State.DONE;
                }
                break;

            case DONE:
                telemetry.addLine("Autonomous complete.");
                break;
        }
        telemetry.update();
    }
}