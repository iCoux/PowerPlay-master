package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOp;
import org.firstinspires.ftc.teamcode.detection.SleeveDetectionLeft;
import org.firstinspires.ftc.teamcode.odometry.util.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.odometry.util.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name ="LeftPreload", group = "Test")
public class LeftPreload extends LinearOpMode {

    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(-35.5, -64.5, Math.toRadians(90));
    Trajectory preload;
    Trajectory parcare;
    OpenCvCamera camera;

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
            telemetry.addLine("boro e sefu pe autonomne");
            telemetry.update();


        }
        waitForStart();




        if(opModeIsActive() && !isStopRequested()) {

            if(sleeveDetection.getPosition()== SleeveDetectionLeft.ParkingPosition.LEFT){
                stacking(-60, -14);
            }else if(sleeveDetection.getPosition()== SleeveDetectionLeft.ParkingPosition.CENTER){
                stacking(-36, -14);
            }else if(sleeveDetection.getPosition()== SleeveDetectionLeft.ParkingPosition.RIGHT){
                stacking(-14, -14);
            }
        }

        while(drive.isBusy()){
            telemetry();

            telemetry.update();
        }

    }
    public void stacking(double x , double y){

        preload = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-36, -11.5, Math.toRadians(57)))
                .addTemporalMarker(0.1, () -> {
                    teleop.vertical.High();
                })
                .addSpatialMarker(new Vector2d(-36 , -11.5) ,()-> {
                    placeCone();
                })
                .build();

        parcare = drive.trajectoryBuilder(preload.end())
                .lineToSplineHeading(new Pose2d(x, y, Math.toRadians(357)))
                .addTemporalMarker(0.3, () -> {
                    parkingRetract();
                })
                .build();

        drive.followTrajectory(preload);
        sleep(1000);
        drive.followTrajectory(parcare);
        telemetry();
        drive.update();
    }


    public void placeCone(){
        teleop.vertical.outTakeLeft.setPosition(0.9);
        teleop.vertical.outTakeRight.setPosition(0.9);
        teleop.vertical.outTakeClaw.setPosition(0.9);
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