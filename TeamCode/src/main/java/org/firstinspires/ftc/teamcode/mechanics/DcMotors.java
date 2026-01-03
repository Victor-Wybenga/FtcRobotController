package org.firstinspires.ftc.teamcode.mechanics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DcMotors {
    private static final double ROTATION_DAMPENING = 1.0;
    private static final double MAX_SPEED = 1.0;

    public static final double ROBOT_SECONDS_PER_FEET = 0.48;
    public static final double ROBOT_SECONDS_PER_DEGREE = 1.0 / 133.0;
    private final DcMotorEx left_motor, right_motor, shooter_motor;

    private final double left_motor_ticks_per_revolution;
    private final double right_motor_ticks_per_revolution;
    private final Telemetry telemetry;

    public DcMotors(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        shooter_motor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        left_motor = hardwareMap.get(DcMotorEx.class, "left_motor");
        right_motor = hardwareMap.get(DcMotorEx.class, "right_motor");

        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_motor_ticks_per_revolution = left_motor.getMotorType().getTicksPerRev();
        right_motor_ticks_per_revolution = right_motor.getMotorType().getTicksPerRev();
    }

    public void telemetry() {
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Ticks per", "%.2f, %.2f", left_motor_ticks_per_revolution, right_motor_ticks_per_revolution);
        telemetry.addData("Left Motor Encoder", "\n\tRevolutions: %.2f",
                left_motor.getVelocity()
        );
        telemetry.addData("Right Motor Encoder", "\n\tRevolutions: %.2f",
                right_motor.getVelocity()
        );
    }

    public void shoot(double strength) {
        shooter_motor.setPower(strength);
    }

    public void stop_drive() {
        left_motor.setPower(0.0);
        right_motor.setPower(0.0);
    }

    public void drive(double throttle, double rotation) {
        rotation /= ROTATION_DAMPENING;

        final double left_power = Range.clip(
                (throttle + rotation) * MAX_SPEED,
                -1.0,
                1.0
        );

        final double right_power = Range.clip(
                (throttle - rotation) * MAX_SPEED,
                -1.0,
                1.0
        );

        right_motor.setPower(right_power);
        left_motor.setPower(left_power);
    }
}
