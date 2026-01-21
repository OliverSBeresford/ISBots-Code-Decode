package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This robot has a two-motor differential-steered (sometimes called tank or skid steer) drivetrain.
 * With a left and right drive motor.
 * The drive on this robot is controlled in an "Arcade" style, with the left stick Y axis
 * controlling the forward movement and the right stick X axis controlling rotation.
 * This allows easy transition to a standard "First Person" control of a
 * mecanum or omnidirectional chassis.
 *
 * The drive wheels are 96mm diameter traction (Rhino) or omni wheels.
 * They are driven by 2x 5203-2402-0019 312RPM Yellow Jacket Planetary Gearmotors.
 *
 * This robot's main scoring mechanism includes an arm powered by a motor, a "wrist" driven
 * by a servo, and an intake driven by a continuous rotation servo.

 * This OpMode uses the motor's encoder and the RunToPosition method to drive the arm to
 * specific setpoints. These are defined as a number of degrees of rotation away from the arm's
 * starting position.
 *
 * Make sure that the arm is reset into the robot, and the wrist is folded in before
 * you run start the OpMode. The motor's encoder is relative and will move the number of degrees
 * you request it to based on the starting position. So if it starts too high, all the motor
 * setpoints will be wrong.
 *
 * The wrist is powered by a goBILDA Torque Servo (2000-0025-0002).
 *
 * The intake wheels are powered by a goBILDA Speed Servo (2000-0025-0003) in Continuous Rotation mode.
 */


@TeleOp(name = "Test feeder servo")
public class TestFeeder extends RobotUtils {
    boolean prevAButtonState = false;

    @Override
    public void init() {
        startHardware();
        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Spin up to a given velocity, then shoot (only on button press, not held)
        if (gamepad1.a && !prevAButtonState) {
            toggleFeeder();
        }

        prevAButtonState = gamepad1.a;

        // Stop shooter
        if (gamepad1.x) {
            stopShooter();
        }

        // Must be called every loop
        update();

        telemetry.addData("Shooter State", launchState);
        telemetry.addData("Velocity", leftLaunch.getVelocity(AngleUnit.RADIANS));
        telemetry.update();
    }
}
