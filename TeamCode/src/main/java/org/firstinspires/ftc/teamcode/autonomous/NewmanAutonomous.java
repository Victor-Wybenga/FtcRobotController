package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanics.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanics.DcMotors;
import org.firstinspires.ftc.teamcode.pathing.PathPart;
import org.firstinspires.ftc.teamcode.mechanics.ServoMotors;
import org.firstinspires.ftc.teamcode.pathing.PathRotation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

public abstract class NewmanAutonomous extends OpMode {
    private static final double IN_RANGE_CM = 70.0;
    private static final double COURSE_CORRECT_THRESHOLD_DEGREES = 10.0;
    private static final double COURSE_CORRECT_GOAL_DEGREES = 3.0;

    private static final PathPart[] NOT_FOUND_PATH = {
        PathPart.Rotate(PathRotation.LEFT, 45.0)
    };

    protected PathPart[] forward_path;
    protected PathPart[] reverse_path;
    protected RobotTeam robot_team;

    private final ElapsedTime state_time = new ElapsedTime();
    private int path_index = 0;

    private AprilTagWebcam april_tag_webcam;
    private DcMotors dc_motors;
    private ServoMotors servo_motors;
    private RobotState robot_state = RobotState.FORWARD_PATH;

    abstract protected void setup();

    @Override public void init() {
        setup();
        april_tag_webcam = new AprilTagWebcam(hardwareMap, telemetry);
        servo_motors = new ServoMotors(hardwareMap, telemetry);
        dc_motors = new DcMotors(hardwareMap, telemetry);

        telemetry.addData("State", robot_state.name());
    }

    @Override public void start() {
        state_time.reset();
    }

    private void path(PathPart[] path, RobotState done_state) {
        if (path_index >= path.length) change_state(done_state);
        else {
            if (state_time.seconds() >= path[path_index].seconds) {
                path_index++;
                state_time.reset();
            }
            else dc_motors.drive(path[path_index].throttle, path[path_index].rotation);
        }
    }

    private void stop_shoot() {
        servo_motors.set_indexer_servos(0.5);
        servo_motors.set_origin_servo(0.5);
        dc_motors.shoot(0.0);
    }

    private void shoot() {
        dc_motors.shoot(1.0);
        if (state_time.seconds() >= 1.0) {
            servo_motors.set_indexer_servos(1.0);
            servo_motors.set_origin_servo(1.0);
        }
    }

    private void course_correct(AprilTagDetection detection) {
        dc_motors.drive(0.0, 0.05 * -signum(detection.ftcPose.bearing));
    }

    private void change_state(RobotState state) {
        switch (state) {
        case IN_RANGE: case SHOOT: case FINDING:
            dc_motors.stop_drive();
            break;
        case REVERSE_PATH:
            stop_shoot();
            break;
        case STOP:
            stop_shoot();
            dc_motors.stop_drive();
            break;
        }
        path_index = 0;
        state_time.reset();
        robot_state = state;
    }

    @Override public void loop() {
        april_tag_webcam.update();
        Optional<AprilTagDetection> current_detection = april_tag_webcam.get_tag_by_id(robot_team.tag_id());

        telemetry.addData("State", robot_state.name());
        telemetry.addData("State Time", state_time);
        current_detection.ifPresent(d -> april_tag_webcam.telemetry_of(d));

        switch (robot_state) {
        case FORWARD_PATH:
            path(forward_path, RobotState.DRIVE);
            break;
        case DRIVE:
            if (!current_detection.isPresent()) change_state(RobotState.FINDING);
            else if (current_detection.get().ftcPose.range <= IN_RANGE_CM)
                change_state(RobotState.IN_RANGE);
            else if (abs(current_detection.get().ftcPose.bearing) >= COURSE_CORRECT_THRESHOLD_DEGREES)
                change_state(RobotState.COURSE_CORRECT);
            else
                dc_motors.drive(1.0, 0.0);
            break;
        case COURSE_CORRECT:
            if (!current_detection.isPresent()) change_state(RobotState.FINDING);
            else if (abs(current_detection.get().ftcPose.bearing) <= COURSE_CORRECT_GOAL_DEGREES)
                change_state(RobotState.DRIVE);
            else
                course_correct(current_detection.get());
            break;
        case NOT_FOUND:
            path(NOT_FOUND_PATH, RobotState.FINDING);
            break;
        case FINDING:
            if (current_detection.isPresent()) change_state(RobotState.DRIVE);
            else if (state_time.seconds() > 1.0) change_state(RobotState.NOT_FOUND);
            break;
        case IN_RANGE:
            if (current_detection.isPresent() && (abs(current_detection.get().ftcPose.bearing) >= COURSE_CORRECT_THRESHOLD_DEGREES / 2))
                course_correct(current_detection.get());
            else change_state(RobotState.SHOOT);
            break;
        case SHOOT:
            if (state_time.seconds() >= 13.0) change_state(RobotState.REVERSE_PATH);
            else shoot();
            break;
        case REVERSE_PATH:
            path(reverse_path, RobotState.STOP);
            break;
        case STOP:
            break;
        }
    }
}