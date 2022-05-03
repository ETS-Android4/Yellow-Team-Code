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

public class TestingStateOriginal {
    /** Evaluates the moving to original state. */
    public static void evaluate(PhysicalRobotState state) {
        int targetTicks = state.getDistanceFromOrigin();

        state.getMotor().setDirection(DcMotor.Direction.REVERSE);

        // There's gotta be a better way to do this...
        state.getMotor().setPower(0.25f);

        // BUG!
        while (state.getMotor().getCurrentPosition() >= targetTicks) {};

        state.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        state.getMotor().setDirection(DcMotor.Direction.FORWARD);
    }
}
