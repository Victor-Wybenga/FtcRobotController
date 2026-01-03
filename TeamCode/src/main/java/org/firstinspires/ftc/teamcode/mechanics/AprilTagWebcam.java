package org.firstinspires.ftc.teamcode.mechanics;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AprilTagWebcam {
    private final AprilTagProcessor april_tag_processor;
    private final VisionPortal vision_portal;
    private List<AprilTagDetection> detections = new ArrayList<>();

    private final Telemetry telemetry;

    public AprilTagWebcam(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        april_tag_processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        vision_portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(april_tag_processor)
                .build();
    }

    public void update() {
        detections = april_tag_processor.getDetections();
    }

    public void telemetry_of(AprilTagDetection detection) {
        if (detection.metadata == null) {
            telemetry.addData("Unknown Tag", "\n\tID: %d", detection.id);
            telemetry.addData("\tCenter Pixels", "\n\t\tX: %.0f px\n\t\tY: %.0f px",
                    detection.center.x, detection.center.y
            );
        } else {
            telemetry.addData("Known Tag", "\n\tID: %d", detection.id);
            telemetry.addData("\tPosition", "\n\t\tX: %.1f cm\n\t\tY: %.1f cm\n\t\tZ: %.1f cm",
                    detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z
            );
            telemetry.addData("\tRotation", "\n\t\tPitch: %.1f deg\n\t\tRoll: %.1f deg\n\t\tYaw: %6.1f deg",
                    detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw
            );
            telemetry.addData("\tPolar", "\n\t\tRange: %.1f cm\n\t\tAzimuth: %.1f deg\n\t\tElevation: %.1f deg",
                    detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation
            );
        }
    }

    public List<AprilTagDetection> get_detections() {
        return detections;
    }

    public Optional<AprilTagDetection> get_tag_by_id(int id) {
        return detections
                .stream()
                .filter(detection -> detection.id == id)
                .findFirst();
    }

    public void stop() {
        if (vision_portal != null) vision_portal.close();
    }
}
