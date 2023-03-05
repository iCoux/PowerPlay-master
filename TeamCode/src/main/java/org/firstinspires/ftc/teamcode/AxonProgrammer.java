package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "AxonProgrammer", group = "Test")
public class AxonProgrammer extends LinearOpMode {


    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double positionArm=0.06;
    public static double positionUAD=0.1;
    public ServoImplEx ArmLeft;
    public ServoImplEx ArmRight;
    public Servo UpAndDown;


    public void runOpMode() throws InterruptedException {


        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        ArmRight = hardwareMap.get(ServoImplEx.class, " ArmLeft");
        ArmLeft = hardwareMap.get(ServoImplEx.class, " ArmRight");
        UpAndDown = hardwareMap.get(Servo.class, "UpAndDown");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){

            packet.put("PositionArm", positionArm);
            packet.put("PositionUAD", positionUAD);

            ArmRight.setPosition(positionArm);
            ArmLeft.setPosition(positionArm);
            UpAndDown.setPosition(positionUAD);

            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("Position",ArmRight.getPosition());
            telemetry.addData("Position",ArmLeft.getPosition());
            telemetry.addData("Position",UpAndDown.getPosition());
            telemetry.update();

        }
    }
}
