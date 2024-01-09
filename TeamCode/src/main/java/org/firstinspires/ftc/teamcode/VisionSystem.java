package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import android.util.Size;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumLibrary;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Point;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;


public class VisionSystem  {
    public static AprilTagDetection blankAprilTag = new AprilTagDetection(75,0,0.4f,new Point(),new Point[]{},null,new AprilTagPoseFtc(0,0,0,0,0,0,0,0,0),new AprilTagPoseRaw(0,0,0, new MatrixF(3, 3) {
        @Override public MatrixF emptyMatrix(int numRows, int numCols) {return null;}@Override public float get(int row, int col) {return 0;}@Override public void put(int row, int col, float value) {}
    }),800000000);
    private MotorHardwareMap motors;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private AprilTagProcessor aprilTag;
    public List<AprilTagDetection> detections;
    private VisionPortal visionPortal;
    public AprilTagDetection target = blankAprilTag; //allowing this to be null will break everything, if you want to delete it use VisionSystem.blankAprilTag
    public int pixelNum;
    public Recognition pixelRecog;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {"Pixel"};
    public VisionSystem(Telemetry telem, HardwareMap map){
        telemetry = telem;
        hardwareMap = map;
    }
    //The variable to store our instance of the TensorFlow Object Detection processor.
    private TfodProcessor tfod;
    private AprilTagProcessor buildAprilTagProcessor() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        AprilTagProcessor myAprilTagProcessor;
        myAprilTagProcessorBuilder.setDrawTagID(true);       // Default: true, for all detections.
        myAprilTagProcessorBuilder.setDrawTagOutline(true);  // Default: true, when tag size was provided (thus eligible for pose estimation).
        myAprilTagProcessorBuilder.setDrawAxes(true);        // Default: false.
        myAprilTagProcessorBuilder.setDrawCubeProjection(true);        // Default: false.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        return myAprilTagProcessor;
    }
    private TfodProcessor buildTfodProcessor(){
        TfodProcessor.Builder myTfodProcessorBuilder = new TfodProcessor.Builder();
        TfodProcessor myTfodProcessor;
        myTfodProcessorBuilder.setMaxNumRecognitions(10);  // Max. number of recognitions the network will return
        myTfodProcessorBuilder.setUseObjectTracker(true);  // Whether to use the object tracker
        myTfodProcessorBuilder.setTrackerMaxOverlap((float) 0.2);  // Max. % of box overlapped by another box at recognition time
        myTfodProcessorBuilder.setTrackerMinSize(16);  // Min. size of object that the object tracker will track
        myTfodProcessor = myTfodProcessorBuilder.build();
        return myTfodProcessor;
    }
    private VisionPortal buildVisionPortal(AprilTagProcessor aprilTagProcessor, TfodProcessor tfodProcessor) {
        VisionPortal.Builder myVisionPortalBuilder = new VisionPortal.Builder();
        VisionPortal myVisionPortal;
        myVisionPortalBuilder.setCamera(hardwareMap.get(CameraName.class, "Webcam 1"));      // Other choices are: RC phone camera and "switchable camera name".
        // Add the AprilTag Processor to the VisionPortal Builder.
        myVisionPortalBuilder.addProcessor(aprilTagProcessor);       // An added Processor is enabled by default.
        myVisionPortalBuilder.addProcessor(tfodProcessor);
        // Optional: set other custom features of the VisionPortal (4 are shown here).
        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
        myVisionPortalBuilder.enableLiveView(true);      // Enable LiveView (RC preview).
        myVisionPortalBuilder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.
        if (USE_WEBCAM) {
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        } else {
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        myVisionPortal = myVisionPortalBuilder.build();
        return myVisionPortal;
    }
    public void begin() {
        aprilTag = buildAprilTagProcessor();
        tfod = buildTfodProcessor();
        VisionPortal myVisionPortal = buildVisionPortal(aprilTag, tfod);
    }
//Initialize the TensorFlow Object Detection processor.
    public void update() {
        telemetryTfod();
        telemetryAprilTag();
    }

    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            pixelNum = currentRecognitions.size();
            pixelRecog = recognition;
        }

    }   // end method telemetryTfod()

    private void telemetryAprilTag() {
        detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                target = detection;
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                target = blankAprilTag;
            }
        }
        if(detections.isEmpty()){target=blankAprilTag;}
        for(AprilTagDetection e:detections){
            if(e.metadata!=null){target=e;break;}
        }
        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}