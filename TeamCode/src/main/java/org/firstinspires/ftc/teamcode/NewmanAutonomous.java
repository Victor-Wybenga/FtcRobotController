package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanics.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanics.DcMotors;
import org.firstinspires.ftc.teamcode.mechanics.ServoMotors;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

/*
 * TODO:
 *
 */


enum RobotState {RUN, TURN, NOT_FOUND, IN_RANGE, SHOOT, STOP}

@Autonomous(name = "Newman Autonomous", group = "Newman")
public class NewmanAutonomous extends OpMode {
    private AprilTagWebcam aprilTagWebcam;

    private final ElapsedTime shoot_time = new ElapsedTime();
    private DcMotors dcMotors;
    private ServoMotors servoMotors;
    private RobotState state = RobotState.RUN;
    private Optional<AprilTagDetection> currentDetection;

    @Override public void init() {
        aprilTagWebcam = new AprilTagWebcam(hardwareMap, telemetry);
        servoMotors = new ServoMotors(hardwareMap, telemetry);
        dcMotors = new DcMotors(hardwareMap, telemetry);

        aprilTagWebcam.update();
        currentDetection = aprilTagWebcam.getTagByID(24);
        telemetry.addData("State", state.name());
        currentDetection.ifPresent(detection -> aprilTagWebcam.telemetry_of(detection));
    }

    @Override public void loop() {
        aprilTagWebcam.update();
        currentDetection = aprilTagWebcam.getTagByID(24);

        telemetry.addData("State", state.name());
        currentDetection.ifPresent(detection -> aprilTagWebcam.telemetry_of(detection));

        switch (state) {
            case RUN:
                if (!currentDetection.isPresent()) state = RobotState.NOT_FOUND;
                else if (currentDetection.get().ftcPose.range <= 80.0) state = RobotState.IN_RANGE;
                else if (abs(currentDetection.get().ftcPose.bearing) >= 10.0) state = RobotState.TURN;
                else dcMotors.drive(-1.0, 0.0);
                break;
            case TURN:
                if (!currentDetection.isPresent()) state = RobotState.NOT_FOUND;
                else if (abs(currentDetection.get().ftcPose.bearing) <= 3.0) state = RobotState.RUN;
                else dcMotors.drive(0.0, 0.1 * -signum(currentDetection.get().ftcPose.bearing));
                break;
            case NOT_FOUND:
                if (currentDetection.isPresent()) state = RobotState.RUN;
                else dcMotors.drive(0.0, 0.01);
                break;
            case IN_RANGE:
                dcMotors.drive(0.0, 0.0);
                shoot_time.reset();
                state = RobotState.SHOOT;
                break;
            case SHOOT:
                dcMotors.drive(0.0, 0.0);
                if (shoot_time.seconds() >= 8.0) state = RobotState.STOP;
                else {
                    servoMotors.set_indexer_servos(1.0);
                    servoMotors.set_origin_servo(1.0);
                    dcMotors.shoot(1.0);
                }
                break;
            case STOP:
                dcMotors.drive(0.0, 0.0);
                servoMotors.set_indexer_servos(0.5);
                servoMotors.set_origin_servo(0.5);
                dcMotors.shoot(0.0);
                break;
        }
    }
}
