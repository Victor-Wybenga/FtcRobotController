package org.firstinspires.ftc.teamcode.mechanics;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoMotors {

    private final Servo origin_servo, left_indexer_servo, right_indexer_servo;
    private final Telemetry telemetry;

    public ServoMotors(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        origin_servo = hardwareMap.get(Servo.class, "origin_servo");
        left_indexer_servo = hardwareMap.get(Servo.class, "left_indexer_servo");
        right_indexer_servo = hardwareMap.get(Servo.class, "right_indexer_servo");
        left_indexer_servo.setDirection(Servo.Direction.REVERSE);
    }

    public void set_indexer_servos(double position) {
        left_indexer_servo.setPosition(position);
        right_indexer_servo.setPosition(position);
    }

    public void set_origin_servo(double position) {
        origin_servo.setPosition(position);
    }
}
