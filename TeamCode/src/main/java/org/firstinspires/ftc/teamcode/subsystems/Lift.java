package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift{

    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;
    public Servo outTakeRight;
    public Servo outTakeLeft;
    public Servo outTakeClaw;
    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;
    public static double Kp = 0.01;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    public static double targetPosition = 0;
    public double outTakeUp = 0.15;
    public double outTakeDown = 0.074;
    public double closedClaw = 0.75;
    public double openedClaw = 0.54;
    public int medium = 480;
    public int high = 1360;

    double currentPosition=0;

    public void initLiftAuto(HardwareMap hardwareMap){

        outTakeRight = hardwareMap.get(Servo.class, " outTakeRight");
        outTakeLeft = hardwareMap.get(Servo.class, " outTakeLeft");
        outTakeClaw = hardwareMap.get(Servo.class, " outTakeClaw");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outTakeClaw.setPosition(0.8);
        outTakeClaw.setPosition(closedClaw);
        outTakeLeft.setPosition(1.02-outTakeDown);
        outTakeRight.setPosition(outTakeDown);
    }

    public void initLiftTeleOp(HardwareMap hardwareMap){

        outTakeRight = hardwareMap.get(Servo.class, " outTakeRight");
        outTakeLeft = hardwareMap.get(Servo.class, " outTakeLeft");
        outTakeClaw = hardwareMap.get(Servo.class, " outTakeClaw");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        outTakeClaw.setPosition(0.55); // transfer pos
        outTakeLeft.setPosition(1.02-outTakeDown);
        outTakeRight.setPosition(outTakeDown);
    }

    public void Low(){
        outTakeLeft.setPosition(0.03);
        outTakeRight.setPosition(0.99);
    }

    public void OutTakeUp(){
        outTakeLeft.setPosition(outTakeUp);
        outTakeRight.setPosition(1.02-outTakeUp);
    }

    public void midPreload(){
        outTakeClaw.setPosition(closedClaw);
        outTakeLeft.setPosition(0);
        outTakeRight.setPosition(1.02);

        liftLeft.setTargetPosition(975);
        liftRight.setTargetPosition(975);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);
    }

    public void OutTakeDown(){
        outTakeClaw.setPosition(0.55);
        outTakeLeft.setPosition(1.02-outTakeDown);
        outTakeRight.setPosition(outTakeDown);
    }
    public void downLift() {

        outTakeClaw.setPosition(0.55);
        outTakeLeft.setPosition(1.02-outTakeDown);
        outTakeRight.setPosition(outTakeDown);

        liftLeft.setTargetPosition(0);
        liftRight.setTargetPosition(0);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(0.9);
        liftRight.setPower(0.9);
    }

    public void Medium() {

        outTakeClaw.setPosition(closedClaw);
        OutTakeUp();

        liftLeft.setTargetPosition(medium);
        liftRight.setTargetPosition(medium);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);

    }
    public void High() {

        outTakeClaw.setPosition(0.75);
        outTakeLeft.setPosition(outTakeUp);
        outTakeRight.setPosition(1.02-outTakeUp);

        liftLeft.setTargetPosition(high);
        liftRight.setTargetPosition(high);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);

    }
    public void liftDown(){
        liftLeft.setTargetPosition(900);
        liftRight.setTargetPosition(900);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);
    }

    public double update(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error- lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return  output;
    }
}