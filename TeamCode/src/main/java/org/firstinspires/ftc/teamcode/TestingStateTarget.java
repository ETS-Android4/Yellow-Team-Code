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

public class TestingStateTarget {
    /** Evaluates the moving to target state. */
    public static void evaluate(PhysicalRobotState state) {
        // There's gotta be a better way to do this...
        state.getMotor().setPower(0.25f);
        while (! state.getTouchSensor().isPressed()) {}
        state.getMotor().setPower(0.0f);

        state.setDistanceFromOrigin(state.getMotor().getCurrentPosition());
        state.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
