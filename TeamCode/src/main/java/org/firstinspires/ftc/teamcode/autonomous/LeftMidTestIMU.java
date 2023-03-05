package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TeleOp;
import org.firstinspires.ftc.teamcode.detection.SleeveDetectionLeft;
import org.firstinspires.ftc.teamcode.odometry.util.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.odometry.util.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous(name ="LeftMid", group = "Test")
public class LeftMidTestIMU extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(-34.5, -64.5, Math.toRadians(90));
    Pose2d placeCone = new Pose2d( -35.8,-17, Math.toRadians(323));
    Trajectory preload;
    Trajectory firstCone;
    Trajectory traj3;
    Trajectory traj4;
    Trajectory traj5;
    Trajectory traj6;
    Trajectory traj7;
    Trajectory traj8;
    Trajectory traj9;
    Trajectory traj10;
    Trajectory traj11;
    Trajectory parcare, preloadMid, preloadMidExtra;
    OpenCvCamera camera;


    private SleeveDetectionLeft sleeveDetection;
    private String webcamName = "Webcam 1";
    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        teleop.vertical.initLiftAuto(hardwareMap);
        teleop.intake.initIntake(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

        preloadMid = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-40,-31.5, Math.toRadians(27)))
                .addTemporalMarker(0.1, () -> {
                    teleop.vertical.midPreload();
                })
                .addSpatialMarker(new Vector2d(-39, -33) ,()-> {
                    dropCone();
                })
                .build();
        preloadMidExtra = drive.trajectoryBuilder(preloadMid.end())
                .lineToSplineHeading(new Pose2d(-37, -14, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.2, () -> {
                    teleop.vertical.downLift();
                })
                .build();
        firstCone = drive.trajectoryBuilder(preloadMidExtra.end())
                .lineToSplineHeading(new Pose2d(-43.1, -13.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.7 ,()->{
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(-43.1 ,-13.5),() -> {
                    collectFirst();
                })
                .build();
        traj3 = drive.trajectoryBuilder(firstCone.end())
                .lineToSplineHeading(placeCone,SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.45 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.7 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.7);
                })
                .addTemporalMarker(0.9,()->{
                    teleop.vertical.Medium();
                })
                .build();
        traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-43.1, -13.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.7,()-> {
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(-43.1 ,-13.5),() ->{
                    collectSecond();
                })
                .build();
        traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(placeCone,SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.45 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.7 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.7);
                })
                .addTemporalMarker(0.9 ,()->{
                    teleop.vertical.Medium();
                })
                .build();
        traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(-42.5, -13.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.7,()-> {
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(-42.5, -13.5), () -> {
                    collectThird();
                })
                .build();
        traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(placeCone,SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.45 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.7 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.7);
                })
                .addTemporalMarker(0.9,()->{
                    teleop.vertical.Medium();
                })
                .build();
        traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(-41.8, -13.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.7, () -> {
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(-41.8 ,-13.5),() -> {
                    collectFourth();
                })
                .build();
        traj9 = drive.trajectoryBuilder(traj8.end())
                .lineToSplineHeading(placeCone,SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.45 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.7 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.7);
                })
                .addTemporalMarker(0.9,()->{
                    teleop.vertical.Medium();
                })
                .build();

        traj10 = drive.trajectoryBuilder(traj9.end())
                .lineToSplineHeading(new Pose2d(-41.3, -13.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.7, () -> {
                    teleop.vertical.downLift();
                })
                .addSpatialMarker(new Vector2d(-41.3 ,-13.5),() -> {
                    collectFifth();
                })
                .build();
        traj11 = drive.trajectoryBuilder(traj10.end())
                .lineToSplineHeading(placeCone,SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.45 ,()->{
                    teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
                })
                .addTemporalMarker(0.7 ,()->{
                    teleop.vertical.outTakeClaw.setPosition(0.7);
                })
                .addTemporalMarker(0.9 ,()->{
                    teleop.vertical.Medium();
                })
                .build();
        parcare = drive.trajectoryBuilder(traj11.end())
                .lineToSplineHeading(new Pose2d(x, y, Math.toRadians(0)))
                .addTemporalMarker(0.1, () -> {
                    parkingRetract();
                })
                .build();

        drive.followTrajectory(preloadMid);
        sleep(500);
        drive.setPoseEstimate(preloadMid.end().plus(new Pose2d(0,0,63 - Math.abs(Integer.parseInt(getAngle())))));
        drive.followTrajectory(preloadMidExtra);
        drive.setPoseEstimate(preloadMidExtra.end().plus(new Pose2d(0,0,90 - Math.abs(Integer.parseInt(getAngle())))));
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
        drive.setPoseEstimate(firstCone.end().plus(new Pose2d(0,0,90 - Math.abs(Integer.parseInt(getAngle())))));
        drive.followTrajectory(traj3);
        sleep(500);
        placeCone();
        sleep(200);
        drive.setPoseEstimate(traj3.end().plus(new Pose2d(0,0,127 - Math.abs(Integer.parseInt(getAngle())))));
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
        drive.setPoseEstimate(traj4.end().plus(new Pose2d(0,0,90 - Math.abs(Integer.parseInt(getAngle())))));
        drive.followTrajectory(traj5);
        sleep(300);
        placeCone();
        sleep(200);
        drive.setPoseEstimate(traj5.end().plus(new Pose2d(0,0,127 - Math.abs(Integer.parseInt(getAngle())))));
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
        drive.setPoseEstimate(traj6.end().plus(new Pose2d(0,0,90 - Math.abs(Integer.parseInt(getAngle())))));
        drive.followTrajectory(traj7);
        sleep(300);
        placeCone();
        sleep(200);
        drive.setPoseEstimate(traj7.end().plus(new Pose2d(0,0,127 - Math.abs(Integer.parseInt(getAngle())))));
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
        drive.setPoseEstimate(traj8.end().plus(new Pose2d(0,0,90 - Math.abs(Integer.parseInt(getAngle())))));
        drive.followTrajectory(traj9);
        sleep(300);
        placeCone();
        sleep(200);
        drive.setPoseEstimate(traj9.end().plus(new Pose2d(0,0,127 - Math.abs(Integer.parseInt(getAngle())))));
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
        drive.setPoseEstimate(traj10.end().plus(new Pose2d(0,0,90 - Math.abs(Integer.parseInt(getAngle())))));
        drive.followTrajectory(traj11);
        sleep(300);
        placeCone();
        sleep(300);
        drive.setPoseEstimate(traj11.end().plus(new Pose2d(0,0,127 - Math.abs(Integer.parseInt(getAngle())))));
        drive.followTrajectory(parcare);

        telemetry();
        drive.update();
    }


    public void dropCone(){
        teleop.vertical.outTakeClaw.setPosition(0.9);
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

    public String getAngle(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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