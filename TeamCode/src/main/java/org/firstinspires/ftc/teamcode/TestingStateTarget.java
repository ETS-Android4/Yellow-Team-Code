package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TestingStateTarget {
    /** Evaluates the moving to target state. */
    public static void evaluate(PhysicalRobotState state) {
        state.getMotor().setPower(0.2f);
    }
}
