package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Horizontal;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOPAutonom", group = "TEST")

public class TeleOPAutonom extends LinearOpMode {


    ElapsedTime timer = new ElapsedTime();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private double lastError = 0;
    private double integralSum = 0;

    public static double Kp = 0.01;
    public static double KpLeft = 0.01;

    boolean slowForwards, slowBackwards;

    double forward, strafe, rotate;
    double  slowRotateRight, slowRotateLeft ;

    public static double targetPosition = 0;
    public Horizontal intake = new Horizontal();
    private Drivetrain drive = new Drivetrain();
    public Lift vertical = new Lift();

    enum State {
        INIT,
        TRUE,
        FALSE
    }

    enum Stacking {
        ON,

        InProgress,
        OFF,

        ReadyForNext
    }

    enum StackingState {
        Retracted,
        ArmDown,
       IntakeClosed,
       TransferReady,
        IntakeOpenedForTransfer,
       OutTakeClosed,
        LiftUp,
        OutTakeOpened,
        LiftDown,
    }




    public void runOpMode() throws InterruptedException {


        boolean ReadyToTransfer =false;
        boolean ready= false;
        boolean stacking=false;
        boolean bagal=false;
        boolean cone=false;
        State searchingCone = State.FALSE;
        Stacking stackingAUTO = Stacking.OFF;
        StackingState stateofStacking = StackingState.Retracted;
        ElapsedTime transferTime = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stackingAutonomTimer = new ElapsedTime();

        drive.initDrivetrain(hardwareMap);
        vertical.initLiftTeleOp(hardwareMap);
        intake.initIntake(hardwareMap);

        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        targetPosition=0;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            dashboard.sendTelemetryPacket(packet);

            double powerRight = returnPower(targetPosition, vertical.liftRight.getCurrentPosition(),Kp,0,0);
            double powerLeft = returnPower(targetPosition, vertical.liftLeft.getCurrentPosition(), KpLeft, 0 ,0);

            vertical.liftRight.setPower(powerRight);
            vertical.liftLeft.setPower(powerLeft);

            forward = gamepad1.right_stick_y;
            strafe = -gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            slowRotateRight = gamepad1.right_trigger;
            slowRotateLeft = gamepad1.left_trigger;
            slowBackwards = gamepad1.left_bumper;
            slowForwards = gamepad1.right_bumper;

            if (slowBackwards) {
                forward = 0.2;
            } else if (slowForwards) {
                forward = -0.2;
            } else {
                forward = gamepad1.right_stick_y;
            }

            if (slowRotateRight > 0)
                rotate = slowRotateRight * 0.2;
            else if (slowRotateLeft > 0)
                rotate = slowRotateLeft * -0.2;
            else
                rotate = gamepad1.right_stick_x;

            drive.LR.setPower(-forward + strafe + rotate);
            drive.RR.setPower(forward + strafe + rotate);
            drive.RF.setPower(-forward + strafe - rotate);
            drive.LF.setPower(forward + strafe - rotate);

            if(gamepad2.right_bumper){ // lift extension
                intake.intakeExtend();
                searchingCone = State.TRUE;
            }

            if (gamepad1.dpad_right){

                intake.intakeLeft.setPosition(0.1);
                intake.intakeRight.setPosition(0.9);
                intake.ArmRight.setPosition(0.695);
                intake.UpAndDown.setPosition(0.6);
                intake.intakeClaw.setPosition(intake.opened);
                searchingCone=State.TRUE;

            }

            if (gamepad1.dpad_up){

                intake.intakeLeft.setPosition(0.15);
                intake.intakeRight.setPosition(0.85);
                intake.ArmRight.setPosition(0.7);
                intake.UpAndDown.setPosition(0.61);
                intake.intakeClaw.setPosition(intake.opened);
                searchingCone=State.TRUE;

            }
            if (gamepad1.dpad_left){

                intake.intakeLeft.setPosition(0.088);
                intake.intakeRight.setPosition(0.9);
                intake.ArmRight.setPosition(0.78);
                intake.UpAndDown.setPosition(0.53);
                intake.intakeClaw.setPosition(intake.opened);
                searchingCone=State.TRUE;
            }

            if (gamepad1.dpad_down){
                intake.intakeLeft.setPosition(0.088);
                intake.intakeRight.setPosition(0.9);
                intake.ArmRight.setPosition(0.85);
                intake.UpAndDown.setPosition(0.5);
                intake.intakeClaw.setPosition(intake.opened);
                searchingCone=State.TRUE;
            }


            if(gamepad2.dpad_right && searchingCone==State.TRUE){
                intake.intakeClaw.setPosition(0.2);
                searchingCone=State.FALSE;
                timer.reset();
                ready=true;
            }

            if(timer.milliseconds()>150 && searchingCone==State.FALSE && ready){
                intake.ArmUpAll();

            }

            if(gamepad2.left_bumper && ready){ //starting transfer
                intake.intakeClaw.setPosition(0);
                transferTime.reset();
                ready=false;
                ReadyToTransfer=true;
            }

            if(transferTime.milliseconds()>150 && ReadyToTransfer){ //cone drop
                vertical.outTakeClaw.setPosition(vertical.closedClaw);
                ReadyToTransfer=false;
                bagal=true;
            }


            if(gamepad2.dpad_left && bagal){
                vertical.outTakeClaw.setPosition(0.4);
                bagal=false;
            }

            if(gamepad1.a){
                intake.ArmRight.setPosition(0.92);
                intake.intakeClaw.setPosition(intake.closed);
                searchingCone=State.TRUE;
            }
            if(gamepad1.b){
                intake.intakeClaw.setPosition(intake.opened);
                searchingCone =State.TRUE;
            }
            if(gamepad1.x){
                intake.intakeClaw.setPosition(intake.closed);

            }

            if(gamepad2.x){
                targetPosition=0;
                vertical.Low();
            }

            if(gamepad2.a){
                targetPosition = 0;
                vertical.OutTakeDown();
            }

            if(gamepad2.b){
                targetPosition = 180;
                vertical.OutTakeUp();
            }

            if(gamepad1.touchpad){
                targetPosition=targetPosition+20;
            }

            if(gamepad2.y){
                targetPosition = 500;
                vertical.OutTakeUp();
            }

            if(gamepad2.dpad_down && searchingCone==State.FALSE){
                searchingCone=State.TRUE;
                intake.ArmDownAll();
                intake.intakeClaw.setPosition(intake.opened);
            }


            if(gamepad2.dpad_up){
                stackingAUTO=Stacking.ON;
            }

            if(stackingAUTO==Stacking.ON || stackingAUTO==Stacking.ReadyForNext){
                intake.ArmDownAll();
                stackingAUTO=Stacking.InProgress;
                stackingAutonomTimer.reset();
                stateofStacking=StackingState.ArmDown;
            }

            if(stackingAutonomTimer.milliseconds()>150 && stateofStacking==StackingState.ArmDown){
                intake.intakeClaw.setPosition(0.3);
                stateofStacking=StackingState.IntakeClosed;
            }

            if(stackingAutonomTimer.milliseconds()>350 && stateofStacking==StackingState.IntakeClosed){
               intake.ArmUpAll();
                stateofStacking=StackingState.TransferReady;
            }


            if(stackingAutonomTimer.milliseconds()>600 && stateofStacking==StackingState.TransferReady){
                intake.intakeClaw.setPosition(0);
                stateofStacking=StackingState.IntakeOpenedForTransfer;
            }

            if(stackingAutonomTimer.milliseconds()>900 && stateofStacking==StackingState.IntakeOpenedForTransfer){
                vertical.outTakeClaw.setPosition(0.75);
                stateofStacking=StackingState.OutTakeClosed;
            }

            if(stackingAutonomTimer.milliseconds()>1300 && stateofStacking==StackingState.OutTakeClosed){
                targetPosition=500;
                stateofStacking=StackingState.LiftUp;
            }

            if(stackingAutonomTimer.milliseconds()>1800 && stateofStacking==StackingState.LiftUp){
                vertical.outTakeClaw.setPosition(0.4);
                stateofStacking=StackingState.OutTakeOpened;
            }

            if(stackingAutonomTimer.milliseconds()>2300 && stateofStacking==StackingState.OutTakeOpened){
                targetPosition=0;
                vertical.OutTakeDown();
                stateofStacking=StackingState.LiftDown;
                stackingAUTO=Stacking.ReadyForNext;

            }














            packet.put("positionRight", vertical.liftRight.getCurrentPosition());
            packet.put("positionLeft", vertical.liftLeft.getCurrentPosition());
            packet.put("errorRight", lastError);
            packet.put("errorLeft", lastError);
            telemetry.addData("Timer:", timer);
            telemetry.update();

            dashboard.sendTelemetryPacket(packet);

        }

    }
    public double returnPower(double reference, double state, double Kp, double Kd, double Ki){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error- lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return  output;
    }
}