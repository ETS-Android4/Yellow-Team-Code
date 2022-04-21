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

public class TestingStateArmDeploy {
    // TODO: Replace value with physical measured value.
    public static final float MINIMUM_ARM_DEPLOY_DISTANCE = 10.0f;

    /** Detect if we still have more to back up in order to
        have the correct amount of space for non-erronious
        opperation. */
    public static void evaluate(PhysicalRobotState state) {
        state.getMotor().setDirection(DcMotor.Direction.REVERSE);
        state.getMotor().setPower(0.25f);
        // PPR!
        while (state.getDistanceSensor().getDistance(DistanceUnit.CM) <= MINIMUM_ARM_DEPLOY_DISTANCE) {}
        state.getMotor().setPower(0.0f);
        state.getMotor().setDirection(DcMotor.Direction.FORWARD);

        state.setDistanceFromOrigin(state.getMotor().getCurrentPosition());
        state.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        state.getServo().setPosition(0.25f);
    }
}

