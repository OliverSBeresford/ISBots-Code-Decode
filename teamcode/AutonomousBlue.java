package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Autonomous(name="Autonomous (Blue Tag 20)")
public class AutonomousBlue extends OpMode {
    private enum State {
        SHOOT_FIRST_BALL,
        INTAKE_BALL,
        SHOOT_SECOND_BALL,
        DONE
    }

    private static final int BLUE_BASKET_TAG_ID = 20;   // change to 24 for red
    private static final double FALLBACK_RPM = 3200;    // used if tag not visible

    private double intakeStartTime = 0.0;
    private double intakeDuration = 2.0; // seconds
    private RobotUtils robot = null;

    private State currentState = State.SHOOT_FIRST_BALL;

    @Override
    public void init() {
        robot = new RobotUtils(hardwareMap);
        telemetry.addLine("Robot Ready.");
        telemetry.addLine("AutoAim: uses AprilTag ID 20 (blue basket).");
        telemetry.update();
    }

    @Override
    public void loop() {
        switch (currentState) {
            case SHOOT_FIRST_BALL:
                robot.shootBallWhenReady();
                currentState = State.INTAKE_BALL;
                break;

            case INTAKE_BALL:
                intakeBall();
                currentState = State.SHOOT_SECOND_BALL;
                break;

            case SHOOT_SECOND_BALL:
                shootSecondBall();
                currentState = State.DONE;
                break;

            case DONE:
                telemetry.addLine("Autonomous complete.");
                telemetry.update();
                break;
        }
        // ===== SHOOT (TAP Y) =====
        robot.shootBallWhenReady(); // your RobotUtils will feed when ready for 2 seconds
        telemetry.addLine("Robot is shooting the 1st ball");
        telemetry.update();
        
        robot.toggle_motor();
        
        intakeWasPressed = true;
        telemetry.addLine("Robot is reloading")
        telemetry.update();

        robot.shootBallWhenReady(); // your RobotUtils will feed when ready for 2 seconds
        telemetry.addLine("Robot is shooting the 2nd ball");
        telemetry.update();
    }
}