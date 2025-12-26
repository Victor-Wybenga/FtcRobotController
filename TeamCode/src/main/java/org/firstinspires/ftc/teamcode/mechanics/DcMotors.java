package org.firstinspires.ftc.teamcode.mechanics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DcMotors {

    private static final double ROTATION_DAMPENING = 1.0;
    private static final double MAX_SPEED = 1.0;

    private static final double AUTO_SPEED = 0.3;
    private final DcMotor left_motor, right_motor, shooter_motor;

    private final double left_motor_ticks_per_revolution;
    private final double right_motor_ticks_per_revolution;
    private final Telemetry telemetry;

    public DcMotors(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        shooter_motor = hardwareMap.get(DcMotor.class, "shooter_motor");
        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        right_motor = hardwareMap.get(DcMotor.class, "right_motor");

        shooter_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_motor_ticks_per_revolution = left_motor.getMotorType().getTicksPerRev();
        right_motor_ticks_per_revolution = right_motor.getMotorType().getTicksPerRev();
    }

    public void shoot(double strength) {
        shooter_motor.setPower(strength);
    }

    public void drive_until(double revolutions, double rotation) {
        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotation /= ROTATION_DAMPENING;

        final double left_target = left_motor_ticks_per_revolution * (revolutions - rotation);
        final double right_target = right_motor_ticks_per_revolution * (revolutions + rotation);

        left_motor.setTargetPosition((int)(left_target));
        right_motor.setTargetPosition((int)(right_target));

        left_motor.setPower(AUTO_SPEED);
        right_motor.setPower(AUTO_SPEED);

        left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean is_drive_busy() {
        return left_motor.isBusy() && right_motor.isBusy();
    }

    public void stop() {
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double throttle, double rotation) {
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
