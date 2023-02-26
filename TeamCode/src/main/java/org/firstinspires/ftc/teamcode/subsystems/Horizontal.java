package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Horizontal {
    public Servo intakeLeft;
    public Servo intakeRight;
    public double extended = 0.9;
    public double retracted =0.54;
    public Servo ArmLeft; // 0.97 down
    public Servo ArmRight; //0.03 down
    public Servo UpAndDown;
    public double ArmUpR = 0.60;

    public double ArmDownR = 0.91;
    public double down = 0.434;

    public double up = 0.198;
    public Servo intakeClaw;

    public double closed = 0.4;
    public double opened = 0.2;


    public void initIntake(HardwareMap hardwareMap){

        intakeLeft = hardwareMap.get(Servo.class, " intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, " intakeRight");
        ArmRight = hardwareMap.get(Servo.class, " ArmRight");
        UpAndDown = hardwareMap.get(Servo.class, " UpAndDown");
        intakeClaw = hardwareMap.get(Servo.class, " intakeClaw");
        intakeClaw.setPosition(opened);
        intakeRetract();
    }
    public void ArmDownAll(){

        ArmRight.setPosition(ArmDownR);
        UpAndDown.setPosition(down);
    }

    public void ArmUpAll(){
        ArmRight.setPosition(ArmUpR);
    }
    public void intakeExtend(){

        intakeLeft.setPosition(0.988-extended);
        intakeRight.setPosition(extended);

        ArmRight.setPosition(ArmDownR);

        UpAndDown.setPosition(down);
        intakeClaw.setPosition(opened);

    }

    public void GroundTerminal(){

        ArmLeft.setPosition(0.09);
        ArmRight.setPosition(0.91);
    }
    public void intakeRetract(){

        intakeLeft.setPosition(0.988-retracted);
        intakeRight.setPosition(retracted);

        ArmRight.setPosition(ArmUpR);

        UpAndDown.setPosition(0.37);
    }

    public void terminal(){

        intakeLeft.setPosition(extended);
        intakeRight.setPosition(extended);

        intakeClaw.setPosition(opened);

        ArmRight.setPosition(ArmDownR);
        UpAndDown.setPosition(down);
    }
}