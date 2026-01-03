/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
                "Game Pad Buttons", "\n\tA: %s\n\tB: %s",
                gamepad1.a ? "Pressed" : "Not Pressed",
                gamepad1.b ? "Pressed" : "Not Pressed"
        );

        servo_motors.set_indexer_servos(gamepad1.a ? 1.0 : 0.5);
        servo_motors.set_origin_servo(gamepad1.a ? 1.0 : 0.5);

        dc_motors.shoot(gamepad1.right_trigger);
        dc_motors.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x);
    }
}
