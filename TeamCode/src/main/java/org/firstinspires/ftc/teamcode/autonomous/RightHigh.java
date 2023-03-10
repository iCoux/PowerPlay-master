package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOp;
import org.firstinspires.ftc.teamcode.detection.SleeveDetectionLeft;
import org.firstinspires.ftc.teamcode.detection.SleeveDetectionRight;
import org.firstinspires.ftc.teamcode.odometry.util.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.odometry.util.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.odometry.util.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name ="RightAutoHigh", group = "Test")
public class RightHigh extends LinearOpMode {

    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(37, -64.5, Math.toRadians(90)); //35.5
    Trajectory preload;
    Trajectory firstCone;
    Trajectory posPreLoad;
    Trajectory traj3;
    Trajectory traj4;
    Trajectory traj5;
    Trajectory traj6;
    Trajectory traj7;
    Trajectory traj8;
    Trajectory traj9;
    Trajectory traj10;
    Trajectory traj11;
    Trajectory parcare;
    OpenCvCamera camera;

    Trajectory parcare1st;




    private SleeveDetectionRight sleeveDetection;
    private String webcamName = "Webcam 1";
    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        teleop.vertical.initLiftAuto(hardwareMap);
        teleop.intake.initIntake(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetectionRight();
        camera.setPipeline(sleeveDetection);

        FtcDashboard.getInstance().startCameraStream(camera, 20);


        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted() && !isStopRequested()) {

            telemetry.addData("Case: ", sleeveDetection.getPosition());
            telemetry.update();


        }
        waitForStart();

        if(opModeIsActive() && !isStopRequested()) {

            if(sleeveDetection.getPosition()== SleeveDetectionRight.ParkingPosition.RIGHT){
                stacking(60, -14);
            }else if(sleeveDetection.getPosition()== SleeveDetectionRight.ParkingPosition.CENTER){
                stacking(36, -14);
            }else if(sleeveDetection.getPosition()== SleeveDetectionRight.ParkingPosition.LEFT){
                stacking(14, -14);
            }

        }

    }
    void tagToTelemetry(AprilTagDetection detection){

        telemetry.addData("CAZ: ", detection.id);
        telemetry.addData("positionLeft", teleop.vertical.liftLeft.getCurrentPosition());
        telemetry.addData("positionRight", teleop.vertical.liftRight.getCurrentPosition());
        telemetry.update();
    }
    public void stacking(double x , double y){



        preload = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(35.2, -12, Math.toRadians(132)))
                .addTemporalMarker(0.1, () -> {
                    teleop.vertical.High();
                })
                .addSpatialMarker(new Vector2d(35.2 , -12) ,()-> {
                    placeCone();
                })
                .build();
        firstCone = drive.trajectoryBuilder(preload.end())
                .lineToSplineHeading(new Pose2d(42, -14.1, Math.toRadians(182)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(42 ,-14.1),() -> {
                    collectFirst();
                })
                .build();
        traj3 = drive.trajectoryBuilder(firstCone.end())
                .lineToSplineHeading(new Pose2d(35.2, -13, Math.toRadians(135)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.5 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.7);
                })
                .addTemporalMarker(0.8 ,()->{
                    teleop.vertical.High();
                })
                .build();
        traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(42.6, -14.1, Math.toRadians(182)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3,()-> {
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(42.6 ,-14.1),() ->{
                    collectSecond();
                })
                .build();
        traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(35.2, -13, Math.toRadians(135)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.5 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.7);
                })
                .addTemporalMarker(0.8 ,()->{
                    teleop.vertical.High();
                })
                .build();
        traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(41.5, -14.1, Math.toRadians(182)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3,()-> {
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(41.5 , -14.1), () -> {
                    collectThird();
                })
                .build();
        traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(35.2, -13, Math.toRadians(135)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.5 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.7);
                })
                .addTemporalMarker(0.8 ,()->{
                    teleop.vertical.High();
                })
                .build();
        traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(40.5, -14.1, Math.toRadians(182)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.7, () -> {
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(40.5 ,-14.1),() -> {
                    collectFourth();
                })
                .build();
        traj9 = drive.trajectoryBuilder(traj8.end())
                .lineToSplineHeading(new Pose2d(35.2, -13, Math.toRadians(135)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.5 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.7);
                })
                .addTemporalMarker(0.8 ,()->{
                    teleop.vertical.High();
                })
                .build();

        traj10 = drive.trajectoryBuilder(traj9.end())
                .lineToSplineHeading(new Pose2d(40.5, -14.1, Math.toRadians(182)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.7, () -> {
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(40.5 ,-14.1),() -> {
                    collectFifth();
                })
                .build();
        traj11 = drive.trajectoryBuilder(traj10.end())
                .lineToSplineHeading(new Pose2d(35.2, -13, Math.toRadians(135)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.5 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.7);
                })
                .addTemporalMarker(0.8 ,()->{
                    teleop.vertical.High();
                })
                .build();
        parcare1st = drive.trajectoryBuilder(traj11.end())
                .lineToSplineHeading(new Pose2d(38,-14 , Math.toRadians(160)))
                .addTemporalMarker(0.1, () -> {
                    parkingRetract();
                })
                .build();
        parcare = drive.trajectoryBuilder(parcare1st.end())
                .lineToSplineHeading(new Pose2d(x, y, Math.toRadians(180)))
                .addTemporalMarker(0.1, () -> {
                    parkingRetract();
                })
                .build();

        drive.followTrajectory(preload);
        sleep(100);
        drive.followTrajectory(firstCone);
        sleep(200);
        grabCone();
        sleep(300);
        UpAndDown();
        //sleep(50);
        //upArm();
        sleep(150);
        retract();
        sleep(200);
        drive.followTrajectory(traj3);
        sleep(500);
        placeCone();
        sleep(200);
        drive.followTrajectory(traj4);
        sleep(200);
        grabCone();
        sleep(300);
        UpAndDown();
        //sleep(50);
        //upArm();
        sleep(100);
        retract();
        sleep(200);
        drive.followTrajectory(traj5);
        sleep(300);
        placeCone();
        sleep(200);
        drive.followTrajectory(traj6);
        sleep(200);
        grabCone();
        sleep(300);
        UpAndDown();
        //sleep(50);
        //upArm();
        sleep(100);
        retract();
        sleep(200);
        drive.followTrajectory(traj7);
        sleep(300);
        placeCone();
        sleep(200);
        drive.followTrajectory(traj8);
        sleep(200);
        grabCone();
        sleep(300);
        UpAndDown();
        //sleep(50);
        //upArm();
        sleep(150);
        retract();
        sleep(200);
        drive.followTrajectory(traj9);
        sleep(300);
        placeCone();
        sleep(200);
        drive.followTrajectory(traj10);
        sleep(200);
        grabCone();
        sleep(300);
        UpAndDown();
        //sleep(50);
        //upArm();
        sleep(100);
        retract();
        sleep(200);
        drive.followTrajectory(traj11);
        sleep(300);
        placeCone();
        sleep(300);
        drive.followTrajectory(parcare);

        telemetry();
        drive.update();
    }


    public void retract(){
        teleop.intake.intakeLeft.setPosition(0.988-teleop.intake.retracted);
        teleop.intake.intakeRight.setPosition(teleop.intake.retracted);
        teleop.intake.ArmLeft.setPosition(0.585);
        teleop.intake.ArmRight.setPosition(0.585);
        teleop.intake.UpAndDown.setPosition(0.03);
    }

    public void upArm (){
        teleop.intake.ArmRight.setPosition(teleop.intake.ArmUp);
        teleop.intake.ArmLeft.setPosition(teleop.intake.ArmUp);
    }

    public void UpAndDown(){
        teleop.intake.UpAndDown.setPosition(0);
    }

    public void grabCone(){
        teleop.intake.intakeClaw.setPosition(teleop.intake.closed);
    }

    public void placeCone(){
        teleop.vertical.outTakeLeft.setPosition(0.9);
        teleop.vertical.outTakeRight.setPosition(0.9);
        teleop.vertical.outTakeClaw.setPosition(0.9);
    }

    public void collectFirst(){
        //teleop.vertical.outTakeLeft.setPosition(1.02- teleop.vertical.outTakeDown);
        //teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
        teleop.intake.intakeLeft.setPosition(0.1);
        teleop.intake.intakeRight.setPosition(0.9);
        teleop.intake.ArmRight.setPosition(0.41);
        teleop.intake.ArmLeft.setPosition(0.41);
        teleop.intake.UpAndDown.setPosition(0.215);
        teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
    }

    public void collectSecond(){
        //teleop.vertical.outTakeLeft.setPosition(1.02- teleop.vertical.outTakeDown);
        //teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
        teleop.intake.intakeLeft.setPosition(0.15);
        teleop.intake.intakeRight.setPosition(0.85);
        teleop.intake.ArmRight.setPosition(0.33);
        teleop.intake.ArmLeft.setPosition(0.33);
        teleop.intake.UpAndDown.setPosition(0.19);
        teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
    }

    public void collectThird(){
        //teleop.vertical.outTakeLeft.setPosition(1.02 - teleop.vertical.outTakeDown);
        //teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
        teleop.intake.intakeLeft.setPosition(0.15);
        teleop.intake.intakeRight.setPosition(0.85);
        teleop.intake.ArmLeft.setPosition(0.24);
        teleop.intake.ArmRight.setPosition(0.24);
        teleop.intake.UpAndDown.setPosition(0.15);
        teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
    }

    public void collectFourth(){
        //teleop.vertical.outTakeLeft.setPosition(1.02 - teleop.vertical.outTakeDown);
        //teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
        teleop.intake.intakeLeft.setPosition(0.15);
        teleop.intake.intakeRight.setPosition(0.85);
        teleop.intake.ArmLeft.setPosition(0.15);
        teleop.intake.ArmRight.setPosition(0.15);
        teleop.intake.UpAndDown.setPosition(0.13);
        teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
    }

    public void collectFifth(){
        //teleop.vertical.outTakeLeft.setPosition(1.02 - teleop.vertical.outTakeDown);
        //teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
        teleop.intake.intakeLeft.setPosition(0.088);
        teleop.intake.intakeRight.setPosition(0.9);
        teleop.intake.ArmLeft.setPosition(0.075);
        teleop.intake.ArmRight.setPosition(0.075);
        teleop.intake.UpAndDown.setPosition(0.1);
        teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
    }

    public void parkingRetract(){
        teleop.vertical.downLift();
        teleop.intake.intakeLeft.setPosition(0.988-teleop.intake.retracted);
        teleop.intake.intakeRight.setPosition(teleop.intake.retracted);
        teleop.intake.ArmRight.setPosition(teleop.intake.ArmUp);
        teleop.intake.ArmLeft.setPosition(teleop.intake.ArmUp);
        teleop.intake.UpAndDown.setPosition(0);
    }


    public void telemetry(){
        telemetry.addData("Left:", teleop.vertical.liftLeft.getCurrentPosition());
        telemetry.addData("Right:", teleop.vertical.liftRight.getCurrentPosition());
        telemetry.addData("X", drive.getPoseEstimate().getX());
        telemetry.addData("Y", drive.getPoseEstimate().getY());
        telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
        telemetry.update();
    }
}