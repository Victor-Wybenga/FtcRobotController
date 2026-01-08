package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.RobotTeam;
import org.firstinspires.ftc.teamcode.mechanics.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanics.DcMotors;
import org.firstinspires.ftc.teamcode.mechanics.ServoMotors;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

@SuppressWarnings("unused")
@TeleOp(name = "Newman TeleOP", group = "Newman")
public class NewmanTeleOp extends OpMode {
    private static final double ROTATION_DAMPENING = 1.5;
    private DcMotors dc_motors;
    private ServoMotors servo_motors;
    private AprilTagWebcam april_tag_webcam;

    @Override public void init() {
        dc_motors = new DcMotors(hardwareMap, telemetry);
        servo_motors = new ServoMotors(hardwareMap, telemetry);
        april_tag_webcam = new AprilTagWebcam(hardwareMap, telemetry);
    }

    @Override public void loop() {
        april_tag_webcam.update();
        Optional<AprilTagDetection> current_detection = april_tag_webcam.get_tag_by_id(RobotTeam.RED.tag_id());
        if(!current_detection.isPresent()) current_detection = april_tag_webcam.get_tag_by_id(RobotTeam.BLUE.tag_id());
        current_detection.ifPresent(d -> april_tag_webcam.telemetry_of(d));

        telemetry.addData("Game Pad Left Stick", "\n\tThrottle: %.2f\n\tDirection: %.2f",
                gamepad1.left_stick_y, gamepad1.left_stick_x
        );

        telemetry.addData("Game Pad Right Trigger", "\n\tPower: %.2f",
                gamepad1.right_trigger
        );

        telemetry.addData(
                "Game Pad Buttons", "\n\tA: %s",
                gamepad1.a ? "Pressed" : "Not Pressed"
        );

        servo_motors.set_indexer_servos(gamepad1.a ? 1.0 : 0.5);
        servo_motors.set_origin_servo(gamepad1.a ? 1.0 : 0.5);

        dc_motors.shoot(gamepad1.right_trigger);
        dc_motors.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x / ROTATION_DAMPENING);
    }
}
