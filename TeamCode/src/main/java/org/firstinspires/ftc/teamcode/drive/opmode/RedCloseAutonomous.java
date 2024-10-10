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
public class RedCloseAutonomous extends LinearOpMode {

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
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    @Override
    public void runOpMode() throws InterruptedException {
        initOpenCV();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(11.68, -61.7, Math.toRadians(90));

        intake = hardwareMap.get(DcMotor.class, "intake");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        leftLift = hardwareMap.get(DcMotor.class, "left_lift");
        rightLift = hardwareMap.get(DcMotor.class, "right_lift");
        leftBox = hardwareMap.get(Servo.class, "left_box");
        rightBox = hardwareMap.get(Servo.class, "right_box");
        holder = hardwareMap.get(Servo.class, "holder");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftLift.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        //sleep(500);
        initialX = cX;
        initialY = cY;

        if (isStopRequested()) return;

        controlHubCam.stopStreaming();
        controlHubCam.closeCameraDevice();

        drive.setPoseEstimate(startPose);
        Trajectory center1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(17, -24, Math.toRadians(-180)), Math.toRadians(-180))
                .addTemporalMarker(1.4 , () -> {
                    intake.setPower(-0.3);
                    leftClaw.setPosition(1);
                    sleep(100);
                    rightClaw.setPosition(0);
                })
                .build();
        Trajectory center2 = drive.trajectoryBuilder(center1.end(), true)
                .splineToConstantHeading(new Vector2d(47, -34), Math.toRadians(0))
                .back(6.5)
                .addTemporalMarker(0, () -> {
                    leftLift.setPower(0.4);
                    rightLift.setPower(0.4);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(0.2, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .build();
        Trajectory center3 = drive.trajectoryBuilder(center2.end())
                .splineToConstantHeading(new Vector2d(16, -10), Math.toRadians(-180))
                .addTemporalMarker(0, () -> {
                    leftBox.setPosition(1);
                    rightBox.setPosition(0);
                })
                .addTemporalMarker(0.8, () -> {
                    leftLift.setPower(-0.7);
                    rightLift.setPower(-0.7);
                    holder.setPosition(0.47);
                })
                .addTemporalMarker(1.3, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-50, -19), Math.toRadians(-180))
                .splineTo(new Vector2d(-66,-19), Math.toRadians(-175),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        Trajectory center4 = drive.trajectoryBuilder(center3.end(),true)
                .splineToConstantHeading(new Vector2d(16, -11), Math.toRadians(0))
                .splineTo(new Vector2d(54, -28), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(1.5, () -> {
                    leftLift.setPower(0.5);
                    rightLift.setPower(0.5);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(1.8, () -> {
                    intake.setPower(0);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .build();
        Trajectory center5 = drive.trajectoryBuilder(center4.end())
                .splineToConstantHeading(new Vector2d(16, -10), Math.toRadians(-180))
                .addTemporalMarker(0, () -> {
                    leftBox.setPosition(1);
                    rightBox.setPosition(0);
                })
                .addTemporalMarker(0.8, () -> {
                    leftLift.setPower(-0.7);
                    rightLift.setPower(-0.7);
                    holder.setPosition(0.47);
                })
                .addTemporalMarker(1.3, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-50, -19), Math.toRadians(-180))
                .splineTo(new Vector2d(-66,-19), Math.toRadians(-175),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        Trajectory center6 = drive.trajectoryBuilder(center5.end(),true)
                .splineToConstantHeading(new Vector2d(16, -11), Math.toRadians(0))
                .splineTo(new Vector2d(54, -28), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(1.5, () -> {
                    leftLift.setPower(0.5);
                    rightLift.setPower(0.5);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(2, () -> {
                    intake.setPower(0);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    leftClaw.setPosition(0);
                    rightClaw.setPosition(1);
                })
                .build();

        Trajectory right1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(30, -30, Math.toRadians(-180)), Math.toRadians(-180))
                .addTemporalMarker(1.5 , () -> {
                    intake.setPower(-0.3);
                    leftClaw.setPosition(1);
                    sleep(100);
                    rightClaw.setPosition(0);
                })
                .build();
        Trajectory right2 = drive.trajectoryBuilder(right1.end(), true)
                .splineToConstantHeading(new Vector2d(47, -42), Math.toRadians(0))
                .back(6.5)
                .addTemporalMarker(0, () -> {
                    leftLift.setPower(0.5);
                    rightLift.setPower(0.5);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(0.2, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .build();
        Trajectory right3 = drive.trajectoryBuilder(right2.end())
                .splineToConstantHeading(new Vector2d(16, -10), Math.toRadians(-180))
                .addTemporalMarker(0, () -> {
                    leftBox.setPosition(1);
                    rightBox.setPosition(0);
                })
                .addTemporalMarker(0.8, () -> {
                    leftLift.setPower(-0.7);
                    rightLift.setPower(-0.7);
                    holder.setPosition(0.47);
                })
                .addTemporalMarker(1.3, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-50, -20), Math.toRadians(-180))
                .splineTo(new Vector2d(-66,-20), Math.toRadians(-175),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        Trajectory right4 = drive.trajectoryBuilder(right3.end(),true)
                .splineToConstantHeading(new Vector2d(16, -11), Math.toRadians(0))
                .splineTo(new Vector2d(54, -31), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(2, () -> {
                    leftLift.setPower(0.5);
                    rightLift.setPower(0.5);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(2.3, () -> {
                    intake.setPower(0);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .build();
        Trajectory right5 = drive.trajectoryBuilder(right4.end())
                .splineToConstantHeading(new Vector2d(16, -10), Math.toRadians(-180))
                .addTemporalMarker(0, () -> {
                    leftBox.setPosition(1);
                    rightBox.setPosition(0);
                })
                .addTemporalMarker(1, () -> {
                    leftLift.setPower(-0.7);
                    rightLift.setPower(-0.7);
                    holder.setPosition(0.47);
                })
                .addTemporalMarker(1.3, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-50, -20), Math.toRadians(-180))
                .splineTo(new Vector2d(-66,-20), Math.toRadians(-175),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        Trajectory right6 = drive.trajectoryBuilder(right5.end(),true)
                .splineToConstantHeading(new Vector2d(16, -11), Math.toRadians(0))
                .splineTo(new Vector2d(54, -31), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(2, () -> {
                    leftLift.setPower(0.5);
                    rightLift.setPower(0.5);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(2.3, () -> {
                    intake.setPower(0);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    leftClaw.setPosition(0);
                    rightClaw.setPosition(1);
                })
                .build();

        Trajectory left1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(9, -32, Math.toRadians(-180)), Math.toRadians(-180))
                .addTemporalMarker(1.5 , () -> {
                    intake.setPower(-0.3);
                    leftClaw.setPosition(1);
                    sleep(100);
                    rightClaw.setPosition(0);
                })
                .build();
        Trajectory left2 = drive.trajectoryBuilder(left1.end(), true)
                .splineToConstantHeading(new Vector2d(47, -25.5), Math.toRadians(0))
                .back(6.5)
                .addTemporalMarker(0, () -> {
                    leftLift.setPower(0.5);
                    rightLift.setPower(0.5);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(0.2, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .build();
        Trajectory left3 = drive.trajectoryBuilder(left2.end())
                .splineToConstantHeading(new Vector2d(16, -10), Math.toRadians(-180))
                .addTemporalMarker(0, () -> {
                    leftBox.setPosition(1);
                    rightBox.setPosition(0);
                })
                .addTemporalMarker(0.8, () -> {
                    leftLift.setPower(-0.7);
                    rightLift.setPower(-0.7);
                    holder.setPosition(0.47);
                })
                .addTemporalMarker(1.3, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-50, -19), Math.toRadians(-180))
                .splineTo(new Vector2d(-66,-19), Math.toRadians(-175),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        Trajectory left4 = drive.trajectoryBuilder(left3.end(),true)
                .splineToConstantHeading(new Vector2d(16, -11), Math.toRadians(0))
                .splineTo(new Vector2d(54, -36), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(2, () -> {
                    leftLift.setPower(0.5);
                    rightLift.setPower(0.5);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(2.3, () -> {
                    intake.setPower(0);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .build();
        Trajectory left5 = drive.trajectoryBuilder(left4.end())
                .splineToConstantHeading(new Vector2d(16, -10), Math.toRadians(-180))
                .addTemporalMarker(0, () -> {
                    leftBox.setPosition(1);
                    rightBox.setPosition(0);
                })
                .addTemporalMarker(1, () -> {
                    leftLift.setPower(-0.7);
                    rightLift.setPower(-0.7);
                    holder.setPosition(0.47);
                })
                .addTemporalMarker(1.3, () -> {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-50, -19), Math.toRadians(-180))
                .splineTo(new Vector2d(-66,-19), Math.toRadians(-175),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        Trajectory left6 = drive.trajectoryBuilder(left5.end(),true)
                .splineToConstantHeading(new Vector2d(16, -11), Math.toRadians(0))
                .splineTo(new Vector2d(54, -36), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(2, () -> {
                    leftLift.setPower(0.5);
                    rightLift.setPower(0.5);
                    leftBox.setPosition(0.3);
                    rightBox.setPosition(0.7);
                })
                .addTemporalMarker(2.5, () -> {
                    intake.setPower(0);
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    leftClaw.setPosition(0);
                    rightClaw.setPosition(1);
                })
                .build();

        if(initialX > 520 || gamepad1.b){
            NeededID = 6;
            right = true;
        }
        else if((initialX > 280 && initialX < 520) || gamepad1.a){
            NeededID = 5;
            center = true;
        }
        else{
            NeededID = 4;
            left = true;
        }
        leftLift.setPower(-0.4);
        rightLift.setPower(-0.4);
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.5);
        holder.setPosition(0.47);


        if(center) {
            drive.followTrajectory(center1);
            drive.followTrajectory(center2);
            holder.setPosition(0.38);
            sleep(300);
            /*
            drive.followTrajectory(center3);
            SetMotorsPower(0.2f);
            eat();
            SetMotorsPower(-0.2f);
            sleep(100);
            SetMotorsPower(0.2f);
            eat();
            SetMotorsPower(0);
            drive.followTrajectory(center4);
            holder.setPosition(0.38);
            sleep(500);
            drive.followTrajectory(center5);
            SetMotorsPower(0.2f);
            eat();
            SetMotorsPower(-0.2f);
            sleep(100);
            SetMotorsPower(0.1f);
            eat();
            SetMotorsPower(0);
            drive.followTrajectory(center6);
            holder.setPosition(0.38);
            sleep(300);

             */
        }
        else if(left) {
            drive.followTrajectory(left1);
            drive.followTrajectory(left2);
            holder.setPosition(0.38);
            sleep(300);
            /*
            drive.followTrajectory(left3);
            SetMotorsPower(0.2f);
            eat();
            SetMotorsPower(-0.2f);
            sleep(100);
            SetMotorsPower(0.2f);
            eat();
            SetMotorsPower(0);
            drive.followTrajectory(left4);
            holder.setPosition(0.38);
            sleep(500);
            drive.followTrajectory(left5);
            SetMotorsPower(0.2f);
            eat();
            SetMotorsPower(-0.2f);
            sleep(100);
            SetMotorsPower(0.1f);
            eat();
            SetMotorsPower(0);
            drive.followTrajectory(left6);
            holder.setPosition(0.38);
            sleep(300);

             */
        }
        else if(right) {
            drive.followTrajectory(right1);
            drive.followTrajectory(right2);
            holder.setPosition(0.38);
            sleep(300);
            /*
            drive.followTrajectory(right3);
            SetMotorsPower(0.2f);
            eat();
            SetMotorsPower(-0.2f);
            sleep(100);
            SetMotorsPower(0.2f);
            eat();
            SetMotorsPower(0);
            drive.followTrajectory(right4);
            holder.setPosition(0.38);
            sleep(500);
            drive.followTrajectory(right5);
            SetMotorsPower(0.2f);
            eat();
            SetMotorsPower(-0.2f);
            sleep(100);
            SetMotorsPower(0.1f);
            eat();
            SetMotorsPower(0);
            drive.followTrajectory(right6);
            holder.setPosition(0.38);
            sleep(300);

             */

        }
        SetMotorsPower(0.3f);
        sleep(300);
        leftFrontDrive.setPower(-0.4);
        leftBackDrive.setPower(0.4);
        rightFrontDrive.setPower(0.4);
        rightBackDrive.setPower(-0.4);
        sleep(2400);
        SetMotorsPower(0);
        sleep(750);
    }

    private void SetMotorsPower(float power){
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    private void eat(){
        leftClaw.setPosition(0.43);
        rightClaw.setPosition(0.57);
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

            Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255);
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
}

