package org.firstinspires.ftc.teamcode.drive.opmode;

import android.view.ViewDebug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.ArrayList;
import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Autonomous(group = "Autonomous")
public class BlueFarAutonomous extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;

    public OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels



    //APRIL TAG VARIABLES


    //OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS



    double tagsize = 0.166667;

    AprilTagDetection tagOfInterest = null;

    //MY VARIABLES
    boolean GetToBackDrop = false;


    double initialX , initialY;
    int NeededID;

    double ticks = 560;
    double newTarget;
    boolean center,right,left;


    double x;
    double z;

    private DcMotor intake = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;
    private Servo leftBox = null;
    private Servo rightBox = null;
    private Servo holder = null;

    float backdropCenter = 38;
    float backdropLeft = 46;
    float backdropRight = 29;



    @Override
    public void runOpMode() throws InterruptedException {
        initOpenCV();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-35.18, 61.7, Math.toRadians(-90));

        intake = hardwareMap.get(DcMotor.class, "intake");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        leftLift = hardwareMap.get(DcMotor.class, "left_lift");
        rightLift = hardwareMap.get(DcMotor.class, "right_lift");
        leftBox = hardwareMap.get(Servo.class, "left_box");
        rightBox = hardwareMap.get(Servo.class, "right_box");
        holder = hardwareMap.get(Servo.class, "holder");

        leftLift.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        //sleep(500);
        initialX = cX;
        initialY = cY;

        if (isStopRequested()) return;

        controlHubCam.stopStreaming();

        drive.setPoseEstimate(startPose);
        Trajectory center1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-36, 32), Math.toRadians(-90))
                .build();
        Trajectory center2 = drive.trajectoryBuilder(center1.end(), true)
                .back(12)
                .build();
        Trajectory center3 = drive.trajectoryBuilder(center2.end())//.plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .splineTo(new Vector2d(-60, 35), Math.toRadians(180))
                .build();
        Trajectory center4 = drive.trajectoryBuilder(center3.end())
                .back(8)
                .build();
        Trajectory center5 = drive.trajectoryBuilder(center4.end())
                .strafeRight(24)
                .addTemporalMarker(0, () -> {
                    intake.setPower(1);
                })
                .build();
        Trajectory center6 = drive.trajectoryBuilder(center5.end(),true)
                .back(60)
                //.splineToConstantHeading(new Vector2d(40, 41.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, backdropCenter), Math.toRadians(0))
                .addTemporalMarker(1.5, () -> {
                    leftLift.setPower(0.5);
                    intake.setPower(0);
                    rightLift.setPower(0.5);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(1.8, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .build();
        Trajectory center7 = drive.trajectoryBuilder(center6.end(),true)
                //.splineToConstantHeading(new Vector2d(55, 41.5), Math.toRadians(0))
                .splineTo(new Vector2d(56, backdropCenter), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        TrajectorySequence center8 = drive.trajectorySequenceBuilder(center7.end())
                //.splineTo(new Vector2d(-58, 38), Math.toRadians(180))
                .splineTo(new Vector2d(-54, 30), Math.toRadians(180))
                .splineTo(new Vector2d(-63,30), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addTemporalMarker(0, () -> {
                    leftBox.setPosition(1);
                    rightBox.setPosition(0);
                })
                .addTemporalMarker(1.2, () -> {
                    leftLift.setPower(-0.7);
                    rightLift.setPower(-0.7);
                    holder.setPosition(0.47);
                })
                .addTemporalMarker(1.7, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .build();
        Trajectory center9 = drive.trajectoryBuilder(center8.end(), true)
                .splineToConstantHeading(new Vector2d(30, 34), Math.toRadians(0))
                .splineTo(new Vector2d(56, 42), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(0, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(1.5, () -> {
                    intake.setPower(0);
                    leftLift.setPower(0.7);
                    rightLift.setPower(0.7);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(1.9, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    leftClaw.setPosition(0);
                    rightClaw.setPosition(1);
                })
                .build();
        Trajectory center10 = drive.trajectoryBuilder(center7.end())
                //.splineToConstantHeading(new Vector2d(55, 41.5), Math.toRadians(0))
                .forward(5)
                .splineToConstantHeading(new Vector2d(45, backdropCenter + 20), Math.toRadians(180))
                .build();

        Trajectory right1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-46, 36), Math.toRadians(-90))
                .build();
        Trajectory right2 = drive.trajectoryBuilder(right1.end(), true)
                .back(10)
                .build();
        Trajectory right3 = drive.trajectoryBuilder(right2.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .splineToConstantHeading(new Vector2d(-60, 34), Math.toRadians(180))
                .build();
        Trajectory right4 = drive.trajectoryBuilder(right3.end())
                .back(5)
                .build();
        Trajectory right5 = drive.trajectoryBuilder(right4.end())
                .strafeRight(26)
                .addTemporalMarker(0, () -> {
                    intake.setPower(1);
                })
                .build();
        Trajectory right6 = drive.trajectoryBuilder(right5.end(),true)
                .back(60)
                //.splineToConstantHeading(new Vector2d(40, 41.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, backdropRight), Math.toRadians(0))
                .addTemporalMarker(1.5, () -> {
                    leftLift.setPower(0.5);
                    intake.setPower(0);
                    rightLift.setPower(0.5);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(1.8, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .build();
        Trajectory right7 = drive.trajectoryBuilder(right6.end(),true)
                //.splineToConstantHeading(new Vector2d(55, 41.5), Math.toRadians(0))
                .splineTo(new Vector2d(58, backdropRight), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        Trajectory right8 = drive.trajectoryBuilder(right7.end())
                //.splineToConstantHeading(new Vector2d(55, 41.5), Math.toRadians(0))
                .forward(5)
                .splineToConstantHeading(new Vector2d(45, backdropRight + 30), Math.toRadians(180))
                .build();

        Trajectory left1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-33, 34), Math.toRadians(0))
                .build();
        Trajectory left2 = drive.trajectoryBuilder(left1.end(), true)
                .back(6)
                .build();
        Trajectory left3 = drive.trajectoryBuilder(left2.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                .splineToConstantHeading(new Vector2d(-60, 34), Math.toRadians(180))
                .build();
        Trajectory left4 = drive.trajectoryBuilder(left3.end())
                .back(5)
                .build();
        Trajectory left5 = drive.trajectoryBuilder(left4.end())
                .strafeRight(26)
                .addTemporalMarker(0, () -> {
                    intake.setPower(1);
                })
                .build();
        Trajectory left6 = drive.trajectoryBuilder(left5.end(),true)
                .back(60)
                //.splineToConstantHeading(new Vector2d(40, 41.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, backdropLeft), Math.toRadians(0))
                .addTemporalMarker(1.5, () -> {
                    leftLift.setPower(0.5);
                    intake.setPower(0);
                    rightLift.setPower(0.5);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(1.8, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .build();
        Trajectory left7 = drive.trajectoryBuilder(left6.end(),true)
                //.splineToConstantHeading(new Vector2d(55, 41.5), Math.toRadians(0))
                .splineTo(new Vector2d(58, backdropLeft), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        Trajectory left8 = drive.trajectoryBuilder(left7.end())
                //.splineToConstantHeading(new Vector2d(55, 41.5), Math.toRadians(0))
                .forward(5)
                .splineToConstantHeading(new Vector2d(45, backdropLeft + 15), Math.toRadians(180))
                .build();

        if(initialX > 520 || gamepad1.b){
            NeededID = 3;
            right = true;
        }
        else if((initialX > 280 && initialX < 520) || gamepad1.a){
            NeededID = 2;
            center = true;
        }
        else{
            NeededID = 1;
            left = true;
        }

        sleep(2000);

        //CENTER BLUE
        if(center) {
            leftLift.setPower(-0.5);
            rightLift.setPower(-0.5);
            leftClaw.setPosition(0.5);
            rightClaw.setPosition(0.5);
            holder.setPosition(0.47);
            drive.followTrajectory(center1);
            leftLift.setPower(0);
            rightLift.setPower(0);
            intake.setPower(-0.3);
            leftClaw.setPosition(1);
            rightClaw.setPosition(0);
            sleep(100);
            intake.setPower(0);
            drive.followTrajectory(center2);
            //drive.turn(Math.toRadians(-90));
            drive.followTrajectory(center3);
            eat();
            //do smth
            drive.followTrajectory(center4);
            drive.followTrajectory(center5);
            drive.followTrajectory(center6);

            initApril();
            sleep(10);
            AprilTag(NeededID);
//        if(x <= -0.35 || x >= 0.35){
//            if(x < -0.35){
//            }
//            else if(x > 0.35){
//            }
//        }
            double xOffset = x * 12;
            telemetry.addLine("Xoffset" + xOffset);
            telemetry.update();
            drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(0, xOffset, 0)));
            GetToBackDrop = true;

            drive.followTrajectory(center7);
            //putToBackdrop();
            holder.setPosition(0.38);
            sleep(400);
            leftLift.setPower(0.4);
            rightLift.setPower(0.4);
            /*
            drive.followTrajectorySequence(center8);
            eat();
            sleep(50);
            eat();
            drive.followTrajectory(center9);
            holder.setPosition(0.38);
            sleep(300);
            //putToBackdrop();

             */
            sleep(200);
            leftLift.setPower(0);
            rightLift.setPower(0);
            leftBox.setPosition(1);
            rightBox.setPosition(0);
            drive.followTrajectory(center10);
            leftClaw.setPosition(0);
            rightClaw.setPosition(1);
        }
        else if(right) {

            //Right
            leftLift.setPower(-0.5);
            rightLift.setPower(-0.5);
            leftClaw.setPosition(0.5);
            rightClaw.setPosition(0.5);
            holder.setPosition(0.47);
            drive.followTrajectory(right1);
            leftLift.setPower(0);
            rightLift.setPower(0);
            intake.setPower(-0.3);
            leftClaw.setPosition(1);
            rightClaw.setPosition(0);
            sleep(100);
            intake.setPower(0);
            drive.followTrajectory(right2);
            drive.turn(Math.toRadians(-90));
            //drive.turn(Math.toRadians(-90));
            drive.followTrajectory(right3);
            eat();
            //do smth
            drive.followTrajectory(right4);
            drive.followTrajectory(right5);
            drive.followTrajectory(right6);

            initApril();
            sleep(10);
            AprilTag(NeededID);
//        if(x <= -0.35 || x >= 0.35){
//            if(x < -0.35){
//            }
//            else if(x > 0.35){
//            }
//        }
            double xOffset = x * 12;
            telemetry.addLine("Xoffset" + xOffset);
            telemetry.update();
            drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(0, xOffset, 0)));
            GetToBackDrop = true;

            drive.followTrajectory(right7);
            holder.setPosition(0.38);
            sleep(400);
            leftLift.setPower(0.3);
            rightLift.setPower(0.3);
            sleep(100);
            leftLift.setPower(0.4);
            rightLift.setPower(0.4);
            sleep(200);
            leftLift.setPower(0);
            rightLift.setPower(0);
            leftBox.setPosition(1);
            rightBox.setPosition(0);
            drive.followTrajectory(right8);
            leftClaw.setPosition(0);
            rightClaw.setPosition(1);
        }
        else if(left) {
            leftLift.setPower(-0.5);
            rightLift.setPower(-0.5);
            leftClaw.setPosition(0.5);
            rightClaw.setPosition(0.5);
            holder.setPosition(0.47);
            drive.followTrajectory(left1);
            leftLift.setPower(0);
            rightLift.setPower(0);
            intake.setPower(-0.3);
            leftClaw.setPosition(1);
            rightClaw.setPosition(0);
            sleep(100);
            intake.setPower(0);
            drive.followTrajectory(left2);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(left3);
            eat();
            //do smth
            drive.followTrajectory(left4);
            drive.followTrajectory(left5);
            drive.followTrajectory(left6);

            initApril();
            sleep(10);
            AprilTag(NeededID);
//        if(x <= -0.35 || x >= 0.35){
//            if(x < -0.35){
//            }
//            else if(x > 0.35){
//            }
//        }
            double xOffset = x * 12;
            telemetry.addLine("Xoffset" + xOffset);
            telemetry.update();
            drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(0, xOffset, 0)));
            GetToBackDrop = true;

            drive.followTrajectory(left7);
            holder.setPosition(0.38);
            sleep(400);
            leftLift.setPower(0.3);
            rightLift.setPower(0.3);
            sleep(100);
            leftLift.setPower(0.4);
            rightLift.setPower(0.4);
            sleep(200);
            leftLift.setPower(0);
            rightLift.setPower(0);
            leftBox.setPosition(1);
            rightBox.setPosition(0);
            drive.followTrajectory(left8);
            leftClaw.setPosition(0);
            rightClaw.setPosition(1);
        }
        sleep(500);

        controlHubCam.stopStreaming();
        controlHubCam.closeCameraDevice();
    }


    private void eat(){
        leftClaw.setPosition(0.42);
        rightClaw.setPosition(0.58);
        intake.setPower(1);
        sleep(800);
        leftClaw.setPosition(1);
        rightClaw.setPosition(0);
        sleep(100);
        intake.setPower(0);
    }

    private void putToBackdrop(){

        sleep(200);
        leftLift.setPower(0);
        rightLift.setPower(0);
        sleep(600);
        holder.setPosition(0.38);
        sleep(500);
        leftLift.setPower(0.4);
        rightLift.setPower(0.4);
        sleep(200);
        leftBox.setPosition(1);
        rightBox.setPosition(0);
        sleep(500);
        leftLift.setPower(-0.6);
        rightLift.setPower(-0.6);
        holder.setPosition(0.47);
        sleep(500);
        leftLift.setPower(0);
        rightLift.setPower(0);
    }
    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());


        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);

        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                controlHubCam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //controlHubCam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(0, 80, 80);
            Scalar upperYellow = new Scalar(40, 255, 255);
            //255 0 0
            //0 0 255


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }

    private void initApril() {
        controlHubCam.closeCameraDevice();

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        controlHubCam.setPipeline(aprilTagDetectionPipeline);


        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);

        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                controlHubCam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        FtcDashboard.getInstance().stopCameraStream();
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
    }


    public void AprilTag(int NeededID)
    {
        /*
        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //controlHubCam.stopStreaming();
                controlHubCam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

         */

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!GetToBackDrop)
        {
            if(isStopRequested()){
                controlHubCam.stopStreaming();
                controlHubCam.closeCameraDevice();
                break;
            }

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == NeededID)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    x = tagOfInterest.pose.x;
                    z = tagOfInterest.pose.z;
                    return;
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
            if(tagOfInterest.pose.x <= 20)
            {
                // do something
            }
            else if(tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50)
            {
                // do something else
            }
            else if(tagOfInterest.pose.x >= 50)
            {
                // do something else
            }
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z));
        telemetry.addLine("X in pixels: " + x);
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}

