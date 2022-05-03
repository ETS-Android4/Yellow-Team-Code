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
                this.state = TestingState.MAKING_ARM_SPACE;

                break;

            case MAKING_ARM_SPACE:
                if (! (robotState.getDistanceSensor().getDistance(DistanceUnit.CM) <=
                        TestingStateArmDeploy.MINIMUM_ARM_DEPLOY_DISTANCE)) {
                    this.state = TestingState.EXTENDING_ARM;

                    robotState.getMotor().setPower(0.0f);
                    robotState.getMotor().setDirection(DcMotor.Direction.FORWARD);

                    robotState.addDistanceFromOrigin(robotState.getMotor().getCurrentPosition());
                    robotState.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    break;
                }

                TestingMakeArmSpace.evaluate(robotState);

                break;

            // Makes sure that we can, and then does extend the arm (with the
            // button on it).
            case EXTENDING_ARM:
                if (robotState.getServo().getPosition() >= TestingStateArmDeploy.ARM_DEPLOY_POSITION) {
                    this.state = TestingState.MOVING_TO_TARGET;

                    break;
                }

                TestingStateArmDeploy.evaluate(robotState);

                break;

            // Move the robot to the target (by querying the button), while keeping track
            // of the distance.
            case MOVING_TO_TARGET:
                if (robotState.getTouchSensor().isPressed()) {
                    this.state = TestingState.MOVING_TO_ORIGINAL;

                    robotState.addDistanceFromOrigin(robotState.getMotor().getCurrentPosition());
                    // This also stops the robot, so we don't need to add that.
                    robotState.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                TestingStateTarget.evaluate(robotState);

                break;

            // Move the robot back the distance that we kept track of in the moving to target
            // state.
            case MOVING_TO_ORIGINAL:
                if (! (robotState.getMotor().getCurrentPosition() >= robotState.getDistanceFromOrigin())) {
                    this.state = TestingState.END;

                    robotState.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robotState.getMotor().setDirection(DcMotor.Direction.FORWARD);
                }

                TestingStateOriginal.evaluate(robotState);

                break;

            default:
                telemetry.addData("Something bad", "Unknown state.");
        }

        // Break out of the state loop if we are in the end state.
        if (this.state == TestingState.END) {
            telemetry.addData("Status", "DIE YOU HEATHEN");
        }

        telemetry.addLine(this.state.toString());

        /* try {
            Thread.sleep(500);
        } catch (InterruptedException ignored) {
            telemetry.addLine("Sleep failed.");
        } */

        telemetry.addLine(robotState.toString());

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    
    }
}
