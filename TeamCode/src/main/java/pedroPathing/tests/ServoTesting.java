package pedroPathing.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;


import pedroPathing.newOld.ControlMotor;

import pedroPathing.newOld.Toggle;
import static pedroPathing.newOld.PositionStorage.*;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "ServoTesting", group = "Linear OpMode")
public class ServoTesting extends LinearOpMode {
    double intakeRotateServoPosition = 30;
    double outakeArmServoPosition = 60;
    double outakeSampleServoPosition = servoextended;
    //double outakeRotateServoPosition =161; // new 0
    double intakeServoPower = 0;

    SparkFunOTOS myOtos;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");
        //Servo outakeRotateServo = hardwareMap.get(Servo.class, "outakeRotateServo");
        DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");

        VisionPortal portal = new VisionPortal.Builder()
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .build();
        dashboardTelemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.



        //Servo tester = hardwareMap.get(Servo.class, "tester");
        ControlMotor intakeControlMotor = new ControlMotor();
        resetStuff();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.right_bumper)
                outakeArmServoPosition += 0.2;
            if (gamepad1.left_bumper)
                outakeArmServoPosition -= 0.2;
            if(gamepad1.a)
                outakeSampleServoPosition--;
            if(gamepad1.b)
                outakeSampleServoPosition++;
            if(gamepad1.x)
                intakeRotateServoPosition+=0.2;
            if(gamepad1.y)
                intakeRotateServoPosition-=0.2;//*/
// 64 deschis 0 inchis la sample
// 0 la rotate 60 pozitie baschet


            if (Toggle.outputtoggle(gamepad1.right_trigger > 0) != 0)
                intakeServoPower = Toggle.outputtoggle(gamepad1.right_trigger > 0);

            //telemetry
            telemetry.addData("outakeArmServoPOS GO TO", outakeArmServoPosition);
            telemetry.addData("outakeArmServoPOS", outakeArmServo.getPosition());
            telemetry.addData("outakeSamplePOS", outakeSampleServo.getPosition());
            telemetry.addData("outakeSamplePOS GO TO ", outakeSampleServoPosition);
            telemetry.addData("intakeRotateServoPosition", intakeRotateServoPosition);
            telemetry.addData("intakeRotateServoPos(TBS)", intakeRotateServo.getPosition());
            telemetry.addData("outake motor pos ", outakeLeftMotor.getCurrentPosition());
            telemetry.addData("intake motor pos ", intakeMotor.getCurrentPosition());
            dashboardTelemetry.addData("outakeArmServoPOS GO TO", outakeArmServoPosition);
            dashboardTelemetry.addData("outakeArmServoPOS", outakeArmServo.getPosition());
            dashboardTelemetry.addData("outakeSamplePOS", outakeSampleServo.getPosition());
            dashboardTelemetry.addData("outakeSamplePOS GO TO ", outakeSampleServoPosition);
            dashboardTelemetry.addData("intakeRotateServoPosition", intakeRotateServoPosition);
            dashboardTelemetry.addData("intakeRotateServoPos(TBS)", intakeRotateServo.getPosition());
            dashboardTelemetry.addData("outake motor pos ", outakeLeftMotor.getCurrentPosition());
            dashboardTelemetry.addData("intake motor pos ", intakeMotor.getCurrentPosition());
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            FtcDashboard.getInstance().startCameraStream(portal, 10);
            dashboardTelemetry.update();
            telemetry.update();



            //tester.setPosition(0);
            intakeRotateServo.setPosition(intakeRotateServoPosition / 360);
            outakeArmServo.setPosition(outakeArmServoPosition / 360);
            outakeSampleServo.setPosition(outakeSampleServoPosition / 360);
            //outakeRotateServo.setPosition(outakeRotateServoPosition/360);


        }

    }
}