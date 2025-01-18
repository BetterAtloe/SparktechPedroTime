package pedroPathing.tests;

import static pedroPathing.States.PositionStorage.IntakeServoColectPos;
import static pedroPathing.States.PositionStorage.PIDincrement;
import static pedroPathing.States.PositionStorage.backLeftPowerCat;
import static pedroPathing.States.PositionStorage.backRightPowerCat;
import static pedroPathing.States.PositionStorage.frontLeftPowerCat;
import static pedroPathing.States.PositionStorage.frontRightPowerCat;
import static pedroPathing.States.PositionStorage.gravityAdder;
import static pedroPathing.States.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.States.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.States.PositionStorage.intakeTargetPos;
import static pedroPathing.States.PositionStorage.isIntakeStateExtended;
import static pedroPathing.States.PositionStorage.isIntakeStateRectracted;
import static pedroPathing.States.PositionStorage.isOuttakeStateSamplePickUp;
import static pedroPathing.States.PositionStorage.isOuttakeStateSpecimen;
import static pedroPathing.States.PositionStorage.outakeArmServoPosition;
import static pedroPathing.States.PositionStorage.outakeSampleRetracted;
import static pedroPathing.States.PositionStorage.outakeSampleServoPosition;
import static pedroPathing.States.PositionStorage.outakeTargetPos;
import static pedroPathing.States.PositionStorage.outakeTargetPosAdder;
import static pedroPathing.States.PositionStorage.servoextended;
import static pedroPathing.States.PositionStorage.wasActivePastActiveIntake;
import static pedroPathing.States.PositionStorage.wasActiveintake;
import static pedroPathing.States.PositionStorage.wasBambuExtended;
import static pedroPathing.States.PositionStorage.wasIntakeStateExtended;
import static pedroPathing.States.PositionStorage.wasOuttakeStateSpecimen;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import pedroPathing.ControlMotor;

@Autonomous(group = "drive")
public class AutonomieDriveTimeSpecimenOnly extends LinearOpMode {
    ExecutorService executorService = Executors.newFixedThreadPool(2);
    final static boolean SLOW = true;
    final float MEEP_MOD = 0.002727f;
    SparkFunOTOS otos;
    private IMU imu = null;
    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    SparkFunOTOS myOtos;
    final float[] hsvValues = new float[3];
    NormalizedColorSensor colorSensor;
    long timer;
    long timer1;

    int a= 0;


    @Override
    public void runOpMode() throws InterruptedException {

        timer = 0;
        // Initialize Outtake states
        /*OuttakeStateSpecimen outtakeSpecimen = new OuttakeStateSpecimen();
        OuttakeSpecimenHang outtakeSpecimenHang = new OuttakeSpecimenHang();
        OuttakeStateBasket outtakeBasket = new OuttakeStateBasket();
        OuttakeStateSamplePickUp outtakeSamplePickUp = new OuttakeStateSamplePickUp();
        OuttakeStateStandbyDownWithSample outtakeStandbyDown = new OuttakeStateStandbyDownWithSample();
        OuttakeStateTranfer outtakeStateTranfer = new OuttakeStateTranfer();
        OuttakeStateStandbyWithSample outtakeStandby = new OuttakeStateStandbyWithSample();
        OutakeHMandWallPU outakeHMandWallPU = new OutakeHMandWallPU();

        // Initialize Intake states
        IntakeStateRetracted intakeRetracted = new IntakeStateRetracted();
        IntakeStateExtended intakeExtended = new IntakeStateExtended();
        IntakeStateWallPURetraction intakeStateWallPURetraction = new IntakeStateWallPURetraction();

        // Create the Outtake FSM with the initial state
        OuttakeFSM outtakeFSM = new OuttakeFSM(outtakeStandbyDown);
        IntakeFSM intakeFSM = new IntakeFSM(intakeRetracted);//*/


        //declare servos

        otos = hardwareMap.get(SparkFunOTOS.class, "SparkFunSensor");
        Config.configureOtos(telemetry, otos);


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        waitForStart();


        otos.begin();
        double otosh = otos.getPosition().h;
        float angle = (float) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (otosh < 0) otosh = otosh + 360;
        imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        imu.getRobotOrientationAsQuaternion();
        imu.getRobotYawPitchRollAngles();




        if (isStopRequested()) return;

        //MultiThread
        executorService.execute(new Runnable() {
            @Override
            public void run() {
                double intakeMotorPower = 0;
                double outakeMotorPower = 0;
                DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
                DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
                DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
                DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
                DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
                DcMotor intakeSpinMotor = hardwareMap.dcMotor.get("intakespin");
                DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
                DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);//*/
                outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                intakeControlMotor = new ControlMotor();
                outakeControlMotor = new ControlMotor();
                //servos
                outakeSampleServoPosition = outakeSampleRetracted;
                Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
                Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
                Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");
                //Servo tester = hardwareMap.get(Servo.class, "tester");
                outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");

                colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");


                while(opModeIsActive()){
                    intakeMotorPower = intakeControlMotor.PIDControl(0, intakeMotor.getCurrentPosition());
                    outakeMotorPower = outakeControlMotor.PIDControlUppy(outakeTargetPos-outakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
                    outakeMotorPower *= PIDincrement;

                    frontLeftMotor.setPower(frontLeftPowerCat);
                    backLeftMotor.setPower(backLeftPowerCat);
                    frontRightMotor.setPower(frontRightPowerCat);
                    backRightMotor.setPower(backRightPowerCat);

                    intakeMotor.setPower(intakeMotorPower);
                    outakeRightMotor.setPower(outakeMotorPower);
                    outakeLeftMotor.setPower(outakeMotorPower);//*/
                    intakeSpinMotor.setPower(intakeMotorPickUpPower);


                    //Set servo Positions
                    intakeRotateServo.setPosition((intakeRotateServoPosition+gravityAdder) / 360);
                    outakeArmServo.setPosition(outakeArmServoPosition / 360);
                    outakeSampleServo.setPosition(outakeSampleServoPosition / 360);
                    telemetry.addData("frontLeftPowerCat",frontLeftPowerCat);
                    telemetry.addData("backLeftPowerCat",backLeftPowerCat);
                    telemetry.addData("frontRightPowerCat",frontRightPowerCat);
                    telemetry.addData("backRightPowerCat",backRightPowerCat);

                    telemetry.addLine("This is Motor "+Thread.currentThread().getId());
                    updateTelemetry(telemetry);
                }
//*/
            }
        });




        //RUN AUTO
        WallPickUp();

        //Hanging Time

        //-go slowly left- HOW TF IS THIS GOING LEFT
        timer = System.currentTimeMillis();
        while(timer + 1000> System.currentTimeMillis()  && opModeIsActive()){}
        timer = System.currentTimeMillis();
        outakeSampleServoPosition = outakeSampleRetracted;

        timer = System.currentTimeMillis();
        while(timer + 1000> System.currentTimeMillis()  && opModeIsActive())
            a = 1;
        OuttakeStateSpecimen();

        //go to bar

        timer = System.currentTimeMillis();
        while(timer+ 700 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  -0.6;
            backLeftPowerCat=  0.6;
            frontRightPowerCat= 0.6;
            backRightPowerCat= -0.6;
        }
        wait(200);
       /* timer = System.currentTimeMillis();
        while(timer+ 1600 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  -0.1;
            backLeftPowerCat=  0.6;
            frontRightPowerCat= 0.6;
            backRightPowerCat= -0.1;
             } */

        timer = System.currentTimeMillis();
        while(timer+ 701 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  0.6;
            backLeftPowerCat=  0.6;
            frontRightPowerCat= 0.6;
            backRightPowerCat= 0.6;
        }

        frontLeftPowerCat=0;
        backLeftPowerCat=0;
        frontRightPowerCat=0;
        backRightPowerCat=0;
        //once at bar
        /*timer = System.currentTimeMillis();
        while(timer + 2000> System.currentTimeMillis()  && opModeIsActive())
            a = 1;//*/
        outakeTargetPos = -1950;

        timer = System.currentTimeMillis();
        while(timer + 300> System.currentTimeMillis()  && opModeIsActive())
            a = 1;
        outakeSampleServoPosition = servoextended;

        timer = System.currentTimeMillis();
        while(timer + 300> System.currentTimeMillis()  && opModeIsActive())
            a = 1;

        //return
        WallPickUp();
        timer = System.currentTimeMillis();
        while(timer+ 700 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  0.6;
            backLeftPowerCat=  -0.6;
            frontRightPowerCat= -0.6;
            backRightPowerCat= 0.6;
        }
        wait(200);

        timer = System.currentTimeMillis();
        while(timer+ 701 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  -0.6;
            backLeftPowerCat=  -0.6;
            frontRightPowerCat= -0.6;
            backRightPowerCat= -0.6;
        }
        //time for second

        timer = System.currentTimeMillis();
        while(timer + 500> System.currentTimeMillis()  && opModeIsActive())
            a = 1;

        frontLeftPowerCat=(0);
        backLeftPowerCat=(0);
        frontRightPowerCat=(0);
        backRightPowerCat=(0);
        outakeSampleServoPosition = outakeSampleRetracted;

        timer = System.currentTimeMillis();
        while(timer + 1200> System.currentTimeMillis()  && opModeIsActive())
            a = 1;
        OuttakeStateSpecimen();

        //go to bar second time

        timer = System.currentTimeMillis();
        while(timer+ 700 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  -0.6;
            backLeftPowerCat=  0.6;
            frontRightPowerCat= 0.6;
            backRightPowerCat= -0.6;
        }
        wait(200);

        timer = System.currentTimeMillis();
        while(timer+ 701 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  0.6;
            backLeftPowerCat=  0.6;
            frontRightPowerCat= 0.6;
            backRightPowerCat= 0.6;
        }
        frontLeftPowerCat=(0);
        backLeftPowerCat=(0);
        frontRightPowerCat=(0);
        backRightPowerCat=(0);
        /*while(timer + 2000> System.currentTimeMillis()  && opModeIsActive())
            a = 1;//*/
        outakeTargetPos = -1950;

        wait(300);
        outakeSampleServoPosition = servoextended;

        //End of second specimen, time gor 3rd

        timer = System.currentTimeMillis();
        wait(300);

        //return
        WallPickUp();
        timer = System.currentTimeMillis();
        while(timer+ 700 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  0.6;
            backLeftPowerCat=  -0.6;
            frontRightPowerCat= -0.6;
            backRightPowerCat= 0.6;
        }
        wait(200);

        timer = System.currentTimeMillis();
        while(timer+ 701 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  -0.6;
            backLeftPowerCat=  -0.6;
            frontRightPowerCat= -0.6;
            backRightPowerCat= -0.6;
        }
        //time for third

        timer = System.currentTimeMillis();
        wait(500);

        frontLeftPowerCat=(0);
        backLeftPowerCat=(0);
        frontRightPowerCat=(0);
        backRightPowerCat=(0);
        outakeSampleServoPosition = outakeSampleRetracted;

        wait(1200);
        OuttakeStateSpecimen();

        //go to bar third time

        while(timer+ 700 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  -0.6;
            backLeftPowerCat=  0.6;
            frontRightPowerCat= 0.6;
            backRightPowerCat= -0.6;
        }
        wait(200);

        timer = System.currentTimeMillis();
        while(timer+ 701 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  0.6;
            backLeftPowerCat=  0.6;
            frontRightPowerCat= 0.6;
            backRightPowerCat= 0.6;
        }
        frontLeftPowerCat=(0);
        backLeftPowerCat=(0);
        frontRightPowerCat=(0);
        backRightPowerCat=(0);
        /*while(timer + 2000> System.currentTimeMillis()  && opModeIsActive())
            a = 1;//*/
        outakeTargetPos = -1950;

        timer = System.currentTimeMillis();
        wait(300);
        outakeSampleServoPosition = servoextended;




        frontLeftPowerCat=(0);
        backLeftPowerCat=(0);
        frontRightPowerCat=(0);
        backRightPowerCat=(0);


        if (isStopRequested()) return;

    }


    public void OuttakeStateSpecimen(){
        telemetry.addData("OuttakeStateSpecimen",true);
        isOuttakeStateSpecimen = true;
        wasOuttakeStateSpecimen= true;
        outakeSampleServoPosition= outakeSampleRetracted;
        outakeArmServoPosition = 345;
        if (intakeMotorPickUpPower != 1)
            intakeMotorPickUpPower =0;
        outakeTargetPos = -outakeTargetPosAdder-1400;
    }
    public void OuttakeStateSamplePickUp(){ // ONe. . ..OuttakeStateSpecimen
        telemetry.addData("OuttakeStateSamplePickUp",true);
        isOuttakeStateSamplePickUp = true;
        outakeArmServoPosition = 70;
        outakeTargetPos =0;
        outakeSampleServoPosition = servoextended;
        if (intakeMotorPickUpPower != 1)
            intakeMotorPickUpPower = 0;
    }
    public void IntakeExtended(){
        isIntakeStateExtended = true;
        wasIntakeStateExtended = true;
        wasActiveintake = true;
        wasActivePastActiveIntake = true;
        wasBambuExtended = true;
        intakeRotateServoPosition = IntakeServoColectPos;
        if(gravityAdder==0)
            gravityAdder = 7;
        intakeMotorPickUpPower = 0.7;
    }
    public void IntakeOutput(){
        isIntakeStateExtended = true;
        wasIntakeStateExtended = true;
        wasActiveintake = true;
        wasActivePastActiveIntake = true;
        wasBambuExtended = true;
        intakeRotateServoPosition = IntakeServoColectPos;
        if(gravityAdder==0)
            gravityAdder = 7;
        intakeMotorPickUpPower = -0.7;
    }

    public void WallPickUp(){
        gravityAdder = 0;
        isIntakeStateRectracted = true;
        intakeRotateServoPosition = 0;
        intakeTargetPos = 0;
        intakeMotorPickUpPower = 0;
        outakeSampleServoPosition=servoextended;
        outakeArmServoPosition = 30;
        outakeTargetPos = 0;
    }
    public void wait(int timeMiliseconds){
        long wTimer = System.currentTimeMillis();
        while(wTimer+ timeMiliseconds > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat = 0;
            backLeftPowerCat= 0;
            frontRightPowerCat= 0;
            backRightPowerCat= 0;
        }
    }
}
