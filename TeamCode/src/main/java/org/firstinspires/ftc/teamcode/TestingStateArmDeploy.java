package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TestingStateArmDeploy {
    // TODO: Replace value with physical measured value.
    public static final float MINIMUM_ARM_DEPLOY_DISTANCE = 45.72f; // 18 inches, I couldn't find a CM ruler.
    public static final float ARM_DEPLOY_POSITION = 0.25f;

    /** Detect if we still have more to back up in order to
        have the correct amount of space for non-erronious
        opperation. */
    public static void evaluate(PhysicalRobotState state) {
        state.getServo().setPosition(ARM_DEPLOY_POSITION);
    }
}
