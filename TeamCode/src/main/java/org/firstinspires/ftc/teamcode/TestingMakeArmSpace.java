package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TestingMakeArmSpace {
        public static void evaluate(PhysicalRobotState state) {
                state.getMotor().setDirection(DcMotor.Direction.REVERSE);
                state.getMotor().setPower(0.2f);
        }
}
