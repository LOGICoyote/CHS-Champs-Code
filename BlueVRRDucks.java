


package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous

public class BlueVRRDucks extends LinearOpMode {

    OpenCvCamera WebCam;
    Thing myThing;
    static int pos = 0;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor arm = null;
    private DcMotor intake;
    private Servo mailbox;
    private Servo OL;
    private Servo OR;
    private Servo OS;
    //private Rev2mDistanceSensor dist;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        OR = hardwareMap.get(Servo.class, "OR");
        OL = hardwareMap.get(Servo.class, "OL");
        OS = hardwareMap.get(Servo.class, "OS");
        //dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");


        mailbox = hardwareMap.get(Servo.class, "mailbox");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");

        holding();

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        myThing = new Thing();
        WebCam.setPipeline(myThing);
        WebCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

        @Override
            public void onOpened() {
                WebCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
        @Override
            public void onError(int errorCode) {
                }
            });
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, 64, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(
                        //-10, -56 mid
                        new Vector2d(-55, 28),Math.toRadians(-90),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                //.splineTo(new Vector2d(-55, -28),Math.toRadians(90))
                .splineTo(
                        new Vector2d(-25, 26),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(15, () -> {
                    armuphigh();
                })

                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                //-58, -57
                .splineTo(new Vector2d(-55,45),Math.toRadians(90))
                .build();
        Trajectory trajfix1 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(4)
                .build();
        Trajectory trajfix2 = drive.trajectoryBuilder(trajfix1.end())
                .back(8.75)
                .build();
        Trajectory trajfix3 = drive.trajectoryBuilder(trajfix2.end())
                .back(1.4, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(trajfix3.end()/*(-59,55.15*/)
                //make a spline to w/ a ton of points if needed to preserve heading
                .lineToConstantHeading(new Vector2d(-23,55.15))
                //.strafeLeft(9)
               // .splineToConstantHeading(new Vector2d(-30, -55.5),Math.toRadians(-90))
                //.splineToConstantHeading(new Vector2d(-62, -54),Math.toRadians(-90))

                //,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
//        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                .strafeLeft(27)
//                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj5.end())
                .back(7)
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .strafeRight(27)
                //.lineToConstantHeading(new Vector2d(-53, 62.5), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .forward(2)
                .build();
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .addDisplacementMarker( () -> {
                    intake.setPower(0);
                })
                .forward(20)
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .addDisplacementMarker(16, () -> {
                    armuphigh();
                })
                .splineTo(new Vector2d(-29, 25),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end(),true)
                //60,40
                .splineTo(new Vector2d(-60,33), Math.toRadians(180))
                .build();
        TrajectorySequence traj1b = drive.trajectorySequenceBuilder(startPose)
                .splineTo(
                        //-10, -56 mid
                        new Vector2d(-55, 28),Math.toRadians(-90),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineTo(
                        new Vector2d(-31.5, 25),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(15, () -> {
                    armupmid();
                })

                .build();
        Trajectory traj2b = drive.trajectoryBuilder(traj1b.end(), true)
                //-58, -57
                //-62, 54
                .splineTo(new Vector2d(-55,45),Math.toRadians(90))
                .build();
        TrajectorySequence traj1a = drive.trajectorySequenceBuilder(startPose)
                .splineTo(
                        //-55,28
                        new Vector2d(-55, 28),Math.toRadians(-90),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineTo(
                        //-40,23
                        //-35, 27
                        new Vector2d(-37, 27),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(17, () -> {
                    armuplow();
                })

                .build();
        Trajectory traj2a = drive.trajectoryBuilder(traj1a.end(), true)
                //-58, -57
                .splineTo(new Vector2d(-55, 45),Math.toRadians(90))
               // .splineTo(new Vector2d(-62, 54),Math.toRadians(90))
                .build();
        waitForStart();
                telemetry.addData("pos", myThing.position);
                telemetry.addData("box 1 analysis", myThing.anal());
                telemetry.update();

                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             if (myThing.position == Thing.CapPosition.C){
                 drive.followTrajectorySequence(traj1);
                 score();
                 armdown();
                 drive.followTrajectory(traj2);
                 drive.followTrajectory(trajfix1);
                 drive.followTrajectory(trajfix2);
                 drive.followTrajectory(trajfix3);
                 duckspin(3000);
                 intake.setPower(1);
                 drive.followTrajectory(traj5);
                 //drive.followTrajectory(traj6);
                 drive.turn(Math.toRadians(15));
                 drive.turn(Math.toRadians(-15));
                 drive.followTrajectory(traj7);
                 drive.followTrajectory(traj8);
                 drive.followTrajectory(traj9);
                 drive.turn(Math.toRadians(90));
                 drive.turn(Math.toRadians(-90));
                 holding();
                 drive.followTrajectory(traj10);
                 drive.followTrajectory(traj11);
                 score();
                 armdown();
                 drive.followTrajectory(traj12);
                 sleep(999999);

             }
             else if (myThing.position == Thing.CapPosition.B){
                drive.followTrajectorySequence(traj1b);
                score();
                armdown();
                 drive.followTrajectory(traj2b);
                 drive.followTrajectory(trajfix1);
                 drive.followTrajectory(trajfix2);
                 drive.followTrajectory(trajfix3);
                 duckspin(3000);
                 intake.setPower(1);
                 drive.followTrajectory(traj5);
                 //drive.followTrajectory(traj6);
                 drive.turn(Math.toRadians(15));
                 drive.turn(Math.toRadians(-15));
                 drive.followTrajectory(traj7);
                 drive.followTrajectory(traj8);
                 drive.followTrajectory(traj9);
                 drive.turn(Math.toRadians(90));
                 drive.turn(Math.toRadians(-90));
                 holding();
                 drive.followTrajectory(traj10);
                 drive.followTrajectory(traj11);
                 score();
                 armdown();
                 drive.followTrajectory(traj12);
                 sleep(999999);
             }
             else{
                 drive.followTrajectorySequence(traj1a);
                 score();
                 armdown();
                 drive.followTrajectory(traj2a);
                 drive.followTrajectory(trajfix1);
                 drive.followTrajectory(trajfix2);
                 drive.followTrajectory(trajfix3);
                 duckspin(3000);
                 intake.setPower(1);
                 drive.followTrajectory(traj5);
                 //drive.followTrajectory(traj6);
                 drive.turn(Math.toRadians(15));
                 drive.turn(Math.toRadians(-15));
                 drive.followTrajectory(traj7);
                 drive.followTrajectory(traj8);
                 drive.followTrajectory(traj9);
                 drive.turn(Math.toRadians(90));
                 drive.turn(Math.toRadians(-90));
                 holding();
                 drive.followTrajectory(traj10);
                 drive.followTrajectory(traj11);
                 score();
                 armdown();
                 drive.followTrajectory(traj12);
                 sleep(999999);
            }
            }
    private void duckspin (long time){
        intake.setPower(0.2);
        sleep(time);
    }
    private void armuphigh(){
        arm.setTargetPosition(425);
        if (arm.getCurrentPosition() >= 425) {
            arm.setPower(0);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
    }
        private void armupmid(){//550 mid
            arm.setTargetPosition(550);
            if (arm.getCurrentPosition() >= 550) {
                arm.setPower(0);
            }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);}
    private void armuplow(){
        arm.setTargetPosition(650);
        if (arm.getCurrentPosition() >= 650) {
            arm.setPower(0);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);}
    private void dump(){
        mailbox.setPosition(0.4);
    }
    private void accepting(){
        mailbox.setPosition(0);
    }
    private void holding(){
        mailbox.setPosition(0.15);
    }
    private void armdown(){
        arm.setTargetPosition(0);
        if (arm.getCurrentPosition() <= 0) {
            arm.setPower(0);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
        //mailbox.setPosition(1);
    }
    private void score() {
        dump();
        sleep(1000);
        accepting();
    }

    public static class Thing extends OpenCvPipeline {
        public enum CapPosition {
            A,
            B,
            C
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Point TopLeftPoint = new Point(95, 80);
        static final Point TopLeftPoint2 = new Point(285, 80);
        //static final Point TopLeftPoint = new Point(85, 80);
        //static final Point TopLeftPoint2 = new Point(250, 80);
        //static final Point TopLeftPoint3 = new Point(100, 115);
        static final int Region_width = 15;
        static final int Region_height = 60;
        Point region2_pointA = new Point(TopLeftPoint2.x, TopLeftPoint2.y);
        Point region2_pointB = new Point(TopLeftPoint2.x + Region_width, TopLeftPoint2.y + Region_height);
        Point region1_pointA = new Point(TopLeftPoint.x, TopLeftPoint.y);
        Point region1_pointB = new Point(TopLeftPoint.x + Region_width, TopLeftPoint.y + Region_height);
        Mat region1_Cr;
        Mat region2_Cr;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        int avg1;
        int avg2;
        //145
        int iscap = 140;
        private volatile CapPosition position = CapPosition.A;
        void inputToCb(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb,Cr,1);
        }
        @Override
        public void init (Mat firstFrame){
           inputToCb(firstFrame);
            region1_Cr= Cr.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cr= Cr.submat(new Rect(region2_pointA, region2_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avg1 = (int)Core.mean(region1_Cr).val[0];
            avg2 = (int)Core.mean(region2_Cr).val[0];


            Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);
            Imgproc.rectangle(input, region2_pointA, region2_pointB, BLUE, 2);


            position = CapPosition.C;
            if (avg2 > iscap){
                position = CapPosition.B;
            }
            else if (avg1 > iscap){
                position = CapPosition.C;
            }
            else{
            position = CapPosition.A;
            }
            Imgproc.rectangle(input,region1_pointA,region1_pointB, GREEN, 1);
            Imgproc.rectangle(input,region2_pointA,region2_pointB, GREEN, 1);
            return input;
        }
        public CapPosition getAnalysis() {
            return(position) ;
        }
        public int anal() {
            return(avg1) ;
        }


    }

}


