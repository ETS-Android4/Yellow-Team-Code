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

// This could be a singleton but... eh.
public class PhysicalRobotState {
    private ElapsedTime elapsedTime = new ElapsedTime();
    private DcMotor motor = null;
    private DistanceSensor distanceSensor = null;
    private TouchSensor touchSensor = null;

    private int distanceFromOrigin = 0;

    /** Constructs the robot's physical state. Does not initialize parameters. */
    public PhysicalRobotState(DcMotor motor, DistanceSensor distanceSensor, TouchSensor touchSensor) {
        this.motor = motor;
        this.distanceSensor = distanceSensor;
        this.touchSensor = touchSensor;
    }

    public void setDefaults() {
        this.motor.setDirection(DcMotor.Direction.FORWARD);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public ElapsedTime getElapsedTime() {
        return elapsedTime;
    }

    public DcMotor getMotor() {
        return motor;
    }

    public DistanceSensor getDistanceSensor() {
        return distanceSensor;
    }

    public TouchSensor getTouchSensor() {
        return touchSensor;
    }

    public int getDistanceFromOrigin() {
        return distanceFromOrigin;
    }

    public void setDistanceFromOrigin(int distance) {
        distanceFromOrigin = distance;
    }
}

