package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name ="LeftMidAuto", group = "Test")
public class LeftMidAuto extends LinearOpMode {

    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(-35.5, -64.5, Math.toRadians(90));
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
    double xScore=-35.3;
    double yScore=-17.5;
    private SleeveDetectionLeft sleeveDetection;
    private String webcamName = "Webcam 1";
    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        teleop.vertical.initLiftAuto(hardwareMap);
        teleop.intake.initIntake(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetectionLeft();
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

            if(sleeveDetection.getPosition()== SleeveDetectionLeft.ParkingPosition.LEFT){
                stacking(-60, -13,xScore,yScore);
            }else if(sleeveDetection.getPosition()== SleeveDetectionLeft.ParkingPosition.CENTER){
                stacking(-36, -14,xScore,yScore);
            }else if(sleeveDetection.getPosition()== SleeveDetectionLeft.ParkingPosition.RIGHT){
                stacking(-14, -14,xScore,yScore);
            }
        }

    }
    void tagToTelemetry(AprilTagDetection detection){

        telemetry.addData("CAZ: ", detection.id);
        telemetry.addData("positionLeft", teleop.vertical.liftLeft.getCurrentPosition());
        telemetry.addData("positionRight", teleop.vertical.liftRight.getCurrentPosition());
        telemetry.update();
    }
    public void stacking(double x , double y,double xScore,double yScore){

        preload = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-35, -12.8, Math.toRadians(50)))
                .addTemporalMarker(0.1, () -> {
                    teleop.vertical.High();
                })
                .addSpatialMarker(new Vector2d(-35 , -10.8) ,()-> {
                    teleop.vertical.outTakeClaw.setPosition(0.5);
                })
                .build();
        firstCone = drive.trajectoryBuilder(preload.end())
                .lineToSplineHeading(new Pose2d(-44.25, -12.5, Math.toRadians(355)),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(-44.25 ,-12.5),() -> {
                    teleop.vertical.outTakeLeft.setPosition(1.02- teleop.vertical.outTakeDown);
                    teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
                    teleop.intake.intakeLeft.setPosition(0.1);
                    teleop.intake.intakeRight.setPosition(0.9);
                    teleop.intake.ArmRight.setPosition(0.695);
                    teleop.intake.UpAndDown.setPosition(0.6);
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .build();
        traj3 = drive.trajectoryBuilder(firstCone.end())
                .lineToSplineHeading(new Pose2d(-35.3,-17.5, Math.toRadians(320)),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.5 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.8);
                })
                .addTemporalMarker(0.8 ,()->{
                    teleop.vertical.Medium();
                })
                .build();
        traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-44.1, -12.5, Math.toRadians(355)),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3,()-> {
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(-44.1 ,-12.5),() ->{
                    teleop.vertical.outTakeLeft.setPosition(1.02- teleop.vertical.outTakeDown);
                    teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
                    teleop.intake.intakeLeft.setPosition(0.15);
                    teleop.intake.intakeRight.setPosition(0.85);
                    teleop.intake.ArmRight.setPosition(0.70);
                    teleop.intake.UpAndDown.setPosition(0.61);
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .build();
        traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-35.3,-17.5, Math.toRadians(320)),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.5 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.8);
                })
                .addTemporalMarker(0.8 ,()->{
                    teleop.vertical.Medium();
                })
                .build();
        traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(-43, -12.5, Math.toRadians(355)),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3,()-> {
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(-43 , -12.5), () -> {
                    teleop.vertical.outTakeLeft.setPosition(1.02 - teleop.vertical.outTakeDown);
                    teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
                    teleop.intake.intakeLeft.setPosition(0.15);
                    teleop.intake.intakeRight.setPosition(0.85);
                    teleop.intake.ArmRight.setPosition(0.78);
                    teleop.intake.UpAndDown.setPosition(0.54);
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .build();
        traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(-35.3,-17.5, Math.toRadians(320)),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.5 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.8);
                })
                .addTemporalMarker(0.8 ,()->{
                    teleop.vertical.Medium();
                })
                .build();
        traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(-41.15, -12.5, Math.toRadians(355)),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.7, () -> {
                    teleop.vertical.downLift();
                    teleop.vertical.outTakeLeft.setPosition(1.02 - teleop.vertical.outTakeDown);
                    teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
                    teleop.intake.intakeLeft.setPosition(0.15);
                    teleop.intake.intakeRight.setPosition(0.85);
                    teleop.intake.ArmRight.setPosition(0.85);
                    teleop.intake.UpAndDown.setPosition(0.5);
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .build();
        traj9 = drive.trajectoryBuilder(traj8.end())
                .lineToSplineHeading(new Pose2d(-35.3,-17.5, Math.toRadians(320)),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.5 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.8);
                })
                .addTemporalMarker(0.8 ,()->{
                    teleop.vertical.Medium();
                })
                .build();

        traj10 = drive.trajectoryBuilder(traj9.end())
                .lineToSplineHeading(new Pose2d(-41.35, -12.5, Math.toRadians(355)),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.7, () -> {
                    teleop.vertical.downLift();
                    teleop.vertical.outTakeLeft.setPosition(1.02 - teleop.vertical.outTakeDown);
                    teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
                    teleop.intake.intakeLeft.setPosition(0.088);
                    teleop.intake.intakeRight.setPosition(0.9);
                    teleop.intake.ArmRight.setPosition(0.85);
                    teleop.intake.UpAndDown.setPosition(0.5);
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .build();
        traj11 = drive.trajectoryBuilder(traj10.end())
                .lineToSplineHeading(new Pose2d(-35.3,-17.5, Math.toRadians(320)),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.5 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.8);
                })
                .addTemporalMarker(0.8 ,()->{
                    teleop.vertical.Medium();
                })
                .build();
        parcare = drive.trajectoryBuilder(traj11.end())
                .lineToSplineHeading(new Pose2d(x, y, Math.toRadians(357)))
                .addTemporalMarker(0.1, () -> {
                    teleop.vertical.downLift();
                    teleop.intake.intakeLeft.setPosition(0.988-teleop.intake.retracted);
                    teleop.intake.intakeRight.setPosition(teleop.intake.retracted);
                    teleop.intake.ArmRight.setPosition(teleop.intake.ArmUpR);
                    teleop.intake.UpAndDown.setPosition(0.355);
                })
                .build();

        drive.followTrajectory(preload);
        sleep(100);
        drive.followTrajectory(firstCone);
        sleep(100);
        teleop.intake.intakeClaw.setPosition(teleop.intake.closed);
        teleop.vertical.outTakeClaw.setPosition(0.55);
        sleep(100);
        teleop.intake.ArmRight.setPosition(teleop.intake.ArmUpR);
        teleop.intake.UpAndDown.setPosition(0.355);
        sleep(70);
        teleop.intake.intakeLeft.setPosition(0.988-teleop.intake.retracted);
        teleop.intake.intakeRight.setPosition(teleop.intake.retracted);
        teleop.intake.UpAndDown.setPosition(0.355);
        sleep(200);
        drive.followTrajectory(traj3);
        sleep(500);
        teleop.vertical.outTakeClaw.setPosition(0.4);
        sleep(200);
        drive.followTrajectory(traj4);
        sleep(100);
        teleop.intake.intakeClaw.setPosition(teleop.intake.closed);
        teleop.vertical.outTakeClaw.setPosition(0.55);
        sleep(100);
        teleop.intake.ArmRight.setPosition(teleop.intake.ArmUpR);
        teleop.intake.UpAndDown.setPosition(0.355);
        sleep(70);
        teleop.intake.intakeLeft.setPosition(0.988-teleop.intake.retracted);
        teleop.intake.intakeRight.setPosition(teleop.intake.retracted);
        teleop.intake.UpAndDown.setPosition(0.355);
        sleep(200);
        drive.followTrajectory(traj5);
        sleep(300);
        teleop.vertical.outTakeClaw.setPosition(0.4);
        sleep(200);
        drive.followTrajectory(traj6);
        sleep(100);
        teleop.intake.intakeClaw.setPosition(teleop.intake.closed);
        teleop.vertical.outTakeClaw.setPosition(0.55);
        sleep(100);
        teleop.intake.ArmRight.setPosition(teleop.intake.ArmUpR);
        teleop.intake.UpAndDown.setPosition(0.355);
        sleep(70);
        teleop.intake.intakeLeft.setPosition(0.988-teleop.intake.retracted);
        teleop.intake.intakeRight.setPosition(teleop.intake.retracted);
        teleop.intake.UpAndDown.setPosition(0.355);
        sleep(200);
        drive.followTrajectory(traj7);
        sleep(300);
        teleop.vertical.outTakeClaw.setPosition(0.4);
        sleep(200);
        drive.followTrajectory(traj8);
        sleep(100);
        teleop.intake.intakeClaw.setPosition(teleop.intake.closed);
        teleop.vertical.outTakeClaw.setPosition(0.55);
        sleep(100);
        teleop.intake.ArmRight.setPosition(teleop.intake.ArmUpR);
        teleop.intake.UpAndDown.setPosition(0.355);
        sleep(70);
        teleop.intake.intakeLeft.setPosition(0.988-teleop.intake.retracted);
        teleop.intake.intakeRight.setPosition(teleop.intake.retracted);
        teleop.intake.UpAndDown.setPosition(0.355);
        sleep(200);
        drive.followTrajectory(traj9);
        sleep(300);
        teleop.vertical.outTakeClaw.setPosition(0.4);
        sleep(200);
        drive.followTrajectory(traj10);
        sleep(100);
        teleop.intake.intakeClaw.setPosition(teleop.intake.closed);
        teleop.vertical.outTakeClaw.setPosition(0.55);
        sleep(200);
        teleop.intake.ArmRight.setPosition(teleop.intake.ArmUpR);
        teleop.intake.UpAndDown.setPosition(0.355);
        sleep(70);
        teleop.intake.intakeLeft.setPosition(0.988-teleop.intake.retracted);
        teleop.intake.intakeRight.setPosition(teleop.intake.retracted);
        teleop.intake.UpAndDown.setPosition(0.355);
        sleep(200);
        drive.followTrajectory(traj11);
        sleep(300);
        teleop.vertical.outTakeClaw.setPosition(0.4);
        sleep(300);
        drive.followTrajectory(parcare);
        drive.update();


        telemetry.addData("Left:", teleop.vertical.liftLeft.getCurrentPosition());
        telemetry.addData("Right:", teleop.vertical.liftRight.getCurrentPosition());
        telemetry.update();

    }

}