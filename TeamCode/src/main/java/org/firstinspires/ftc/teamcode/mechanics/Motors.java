package org.firstinspires.ftc.teamcode.mechanics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Motors {

    private static final double ROTATION_DAMPENING = 1.0;
    private static final double MAX_SPEED = 1.0;

    private final DcMotor left_motor, right_motor;

    public Motors(HardwareMap hardwareMap) {
        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        right_motor = hardwareMap.get(DcMotor.class, "right_motor");

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double throttle, double rotation) {
        rotation /= ROTATION_DAMPENING;

        final double left_power = Range.clip(
                (throttle - rotation) * MAX_SPEED,
                -1.0,
                1.0
        );

        final double right_power = Range.clip(
                (throttle + rotation) * MAX_SPEED,
                -1.0,
                1.0
        );

        left_motor.setPower(left_power);
        right_motor.setPower(right_power);
    }
}
