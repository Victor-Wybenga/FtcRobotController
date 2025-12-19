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

    public void telemetry(AprilTagDetection detection) {
        if (detection.metadata == null) {
            telemetry.addData("\n=== Unknown", "(ID: %d) ===", detection.id);
            telemetry.addData("Center", "(X: %6.0f px, Y: %6.0f px)",
                    detection.center.x, detection.center.y
            );
        } else {
            telemetry.addData("\n=== Known", "(ID: %d) ===", detection.id);
            telemetry.addData("Position", "(X: %6.1f cm, Y: %6.1f cm, Z: %6.1f cm)",
                    detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z
            );
            telemetry.addData("Rotation", "(Pitch: %6.1f deg, Roll: %6.1f deg, Yaw: %6.1f deg)",
                    detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw
            );
            telemetry.addData("Polar", "(Range: %6.1f cm, Azimuth: %6.1f deg, Elevation: %6.1f deg)",
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
