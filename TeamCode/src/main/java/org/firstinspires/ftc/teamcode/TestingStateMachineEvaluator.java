package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Basic: First State Machine Evaluation TeleOp.", group="Regular Opmode")
public class TestingStateMachineEvaluator extends OpMode {
    private PhysicalRobotState robotState = null;
    private TestingState state = TestingState.INITIAL;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robotState = new PhysicalRobotState(
            hardwareMap.get(DcMotor.class, "motor"),
            hardwareMap.get(DistanceSensor.class, "distance"),
            hardwareMap.touchSensor.get("touch"),
            hardwareMap.get(Servo.class, "servo")
        );

        robotState.setDefaults();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() { }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robotState.getElapsedTime().reset();
    }

    @Override
    public void loop() {
        // Loop until we reach the end state.
        switch (this.state) {
            // Redirect our starting state from initial to moving to target.
            case INITIAL:
                this.state = TestingState.EXTENDING_ARM;
                break;

            // Makes sure that we can, and then does extend the arm (with the
            // button on it).
            case EXTENDING_ARM:
                TestingStateArmDeploy.evaluate(robotState);
                this.state = TestingState.MOVING_TO_TARGET;
                break;

            // Move the robot to the target (by querying the button), while keeping track
            // of the distance.
            case MOVING_TO_TARGET:
                TestingStateTarget.evaluate(robotState);
                this.state = TestingState.MOVING_TO_ORIGINAL;
                break;

            // Move the robot back the distance that we kept track of in the moving to target
            // state.
            case MOVING_TO_ORIGINAL:
                TestingStateOriginal.evaluate(robotState);
                this.state = TestingState.END;
                break;

            default:
                telemetry.addData("Something bad", "Unknown state.");
        }

        // Break out of the state loop if we are in the end state.
        if (this.state == TestingState.END) {
            telemetry.addData("Status5", "DIE YOU HEATHEN");
        }

        telemetry.addData("state", this.state.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    
    }
}
