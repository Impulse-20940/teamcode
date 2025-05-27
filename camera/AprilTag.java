package org.firstinspires.ftc.teamcode.camera;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
@TeleOp(name = "ApriltagRead")
public class AprilTag extends LinearOpMode {
    private static final boolean USE_WEBCAM = false; //false для использования вебкамеры телефона
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    Telemetry tm;
    @Override
    public void runOpMode() {
        tm = FtcDashboard.getInstance().getTelemetry();
        initAprilTag();
        tm.addData("DS preview on/off", "3 dots, Camera Stream");
        tm.addData(">", "Touch START to start OpMode");
        tm.update();
        waitForStart();
        while (opModeIsActive()) {

            telemetryAprilTag();

            tm.update();

            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }
            sleep(20);
        }

        visionPortal.close();

    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "cam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();


    }
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        tm.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                tm.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                tm.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                tm.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
                tm.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                tm.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        tm.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        tm.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }

}

