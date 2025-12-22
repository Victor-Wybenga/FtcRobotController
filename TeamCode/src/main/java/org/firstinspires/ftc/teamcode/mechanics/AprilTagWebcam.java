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
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private List<AprilTagDetection> detections = new ArrayList<>();

    private final Telemetry telemetry;

    public AprilTagWebcam(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    public void update() {
        detections = aprilTagProcessor.getDetections();
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

    public List<AprilTagDetection> getDetections() {
        return detections;
    }

    public Optional<AprilTagDetection> getTagByID(int id) {
        return detections
                .stream()
                .filter(detection -> detection.id == id)
                .findFirst();
    }

    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }
}
