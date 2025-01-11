package pedroPathing;


import static pedroPathing.States.PositionStorage.*;

import android.graphics.Color;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.ControlMotor;
import pedroPathing.States.IntakeFSM;
import pedroPathing.States.IntakeStateExtended;
import pedroPathing.States.IntakeStateRetracted;
import pedroPathing.States.OutakeHM;
import pedroPathing.States.OuttakeFSM;
import pedroPathing.States.OuttakeSpecimenHang;
import pedroPathing.States.OuttakeStateBasket;
import pedroPathing.States.OuttakeStateSamplePickUp;
import pedroPathing.States.OuttakeStateSpecimen;
import pedroPathing.States.OuttakeStateStandbyDownWithSample;
import pedroPathing.States.OuttakeStateStandbyWithSample;
import pedroPathing.States.PositionStorage;
import pedroPathing.Toggle;
import pedroPathing.tests.Config;


@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "Robot Teleop", group = "Linear OpMode")
public class MainTeleOP extends LinearOpMode {

    final float[] hsvValues = new float[3];

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    SparkFunOTOS myOtos;
    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        PositionStorage.resetStuff();


        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
        DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);//*/

        myOtos = hardwareMap.get(SparkFunOTOS.class, "SparkFunSensor");
        //declare servos
        CRServo servo1 = hardwareMap.get(CRServo.class, "rightservo");
        CRServo servo2 = hardwareMap.get(CRServo.class, "leftservo");
        Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");
        //Servo tester = hardwareMap.get(Servo.class, "tester");
        intakeControlMotor = new ControlMotor();
        outakeControlMotor = new ControlMotor();

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Config.configureOtos(telemetry, myOtos);
        //Thread
        //TempRunnable multiRunnable=new TempRunnable();
        //Thread multiThread = new Thread(multiRunnable);



        //TODO maybe start the Thread        multiThread.start();

        // Initialize Outtake states
        OuttakeStateSpecimen outtakeSpecimen = new OuttakeStateSpecimen();
        OuttakeSpecimenHang outtakeSpecimenHang = new OuttakeSpecimenHang();
        OuttakeStateBasket outtakeBasket = new OuttakeStateBasket();
        OuttakeStateSamplePickUp outtakeSamplePickUp = new OuttakeStateSamplePickUp();
        OuttakeStateStandbyDownWithSample outtakeStandbyDown = new OuttakeStateStandbyDownWithSample();
        OuttakeStateStandbyWithSample outtakeStandby = new OuttakeStateStandbyWithSample();
        OutakeHM outakeHM = new OutakeHM();

        // Initialize Intake states
        IntakeStateRetracted intakeRetracted = new IntakeStateRetracted();
        IntakeStateExtended intakeExtended = new IntakeStateExtended();

        // Create the Outtake FSM with the initial state
        OuttakeFSM outtakeFSM = new OuttakeFSM(outtakeStandbyDown);
        outtakeFSM.executeCurrentState();

        // Create the Intake FSM with the initial state
        IntakeFSM intakeFSM = new IntakeFSM(intakeRetracted);
        intakeFSM.executeCurrentState();

        
        intakeRotateServo.setPosition(intakeRotateServoPosition / 360);
        outakeArmServo.setPosition(outakeArmServoPosition / 360);
        outakeSampleServo.setPosition(outakeSampleServoPosition / 360);
        isOuttakeStateStandbyWithSample = false;
        Config.configureOtos(telemetry, myOtos);

        waitForStart();

        if (isStopRequested()){
            //multiRunnable.stopRunning();
            return;
        }

        while (opModeIsActive()) {
            ///gamepad1
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;
            boolean slowdown = gamepad1.left_bumper;

            ///gamepad2
            double intakeinput = gamepad2.left_stick_y;
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);


            // Slowy downy
            double slowydowny = 3;
            double slowyDownyManal = 3;

            // reversing values
            pivot = -pivot;

            //Arm pos
            armServoPos = outakeArmServo.getPosition() * 360;

            //increment
            PIDincrement = 1;


            //Selectare Echipa
            if (gamepad2.left_bumper && gamepad2.start)
                team = "BLUE";
            if (gamepad2.right_bumper && gamepad2.start)
                team = "RED";

            //just in case
            if (gamepad1.left_bumper && gamepad1.start)
                team = "BLUE";
            if (gamepad1.right_bumper && gamepad1.start)
                team = "RED";


            //Declare colors and team output
            telemetry.addData("Curent team:", team);
            String color = "";
            if (colors.red > colors.blue && colors.red > colors.green)
                color = "BLUE"; //ACTUALLY RED BUT NEEDS INVERSION TO SPIT OUT
            if (colors.blue > colors.red && colors.blue > colors.green)
                color = "RED";  //ACTUALLY BLUE BUT NEEDS INVERSION TO SPIT OUT
            if (colors.green > colors.blue && colors.green > colors.red)
                color = "YELLOW";


            //If bad sample then kick out
            if (colors.red >= 0.007 || colors.blue >= 0.007) {
                if (color.equals(team)) {
                    wasBadSample = true;
                } else {
                    if (wasIntakeStateExtended) {
                        colortimer = System.currentTimeMillis();
                        wasIntakeStateExtended = false;
                    }
                    intakeFSM.setState(intakeRetracted);
                    intakeFSM.executeCurrentState();
                    if(!gamepad2.y)
                        outakeSampleServoPosition = servoextended;
                    shouldTransfer = true;
                }
            }


            ///BEGINNING STATES CODE


            //Intake Button Function for 1A
            if (gamepad1.a)
                isPressedA1 = true;
            if (!gamepad1.a && isPressedA1) {
                if (intakeFSM.currentStateIntake == intakeExtended) {
                    //code state 1
                    intakeFSM.setState(intakeRetracted);
                    intakeFSM.executeCurrentState();
                } else if (intakeFSM.currentStateIntake == intakeRetracted) {
                    //code state 2 ( probabil restras)
                    intakeFSM.setState(intakeExtended);
                    intakeFSM.executeCurrentState();
                } else telemetryOhNo = true;

                isPressedA1 = false;
            }


            //TRANSFER

            //TRANSFER INIT
            if (((intakeMotor.getCurrentPosition() < intakeTargetPosAdder + intakeTransferMarjeOfErrorBeforeTransfer + intakeActualZero && intakeFSM.currentStateIntake == intakeRetracted) || gamepad1.dpad_right)
                    && outtakeFSM.currentStateOutake != outtakeBasket
                    && outtakeFSM.currentStateOutake != outtakeSpecimen
                    && outtakeFSM.currentStateOutake != outtakeSpecimenHang
                    && outtakeFSM.currentStateOutake != outakeHM) {
                if(wasBambuExtended) {
                    bambuTransferTimer = System.currentTimeMillis();
                    wasBambuExtended = false;
                }

                //ACTUAL TRANSFER
                if (bambuTransferTimer + 100 < System.currentTimeMillis()) {
                    intakeServoPower = -0.2;
                    if (colors.red < 0.007 && colors.blue < 0.007) {
                        if (isIntakeStateExtended) {
                            isIntakeStateExtended = false;
                            startingTimer = System.currentTimeMillis();
                        }
                        if (System.currentTimeMillis() > startingTimer + intakeTimeTransferAdder) {
                            outtakeFSM.setState(outtakeStandbyDown);
                            outtakeFSM.executeCurrentState();
                        }
                        if (intakeServoPower != 1 && System.currentTimeMillis() > startingTimer + intakeTimeTransferAdder)
                            intakeServoPower = 0;
                    }
                }
            }



            //Outtake Basket
            if (gamepad1.x)
                isPressedX1 = true;
            if (!gamepad1.x && isPressedX1) {
                if (outtakeFSM.currentStateOutake != outtakeBasket) {
                    outtakeFSM.setState(outtakeBasket);
                    outtakeFSM.executeCurrentState();
                } else if (outtakeFSM.currentStateOutake == outtakeBasket && outakeSampleServoPosition == servoextended) {
                    outtakeFSM.setState(outtakeSamplePickUp);
                    outtakeFSM.executeCurrentState();
                } else if (outtakeFSM.currentStateOutake == outtakeBasket) {
                    outakeSampleServoPosition = servoextended;
                } else telemetryOhNo = true;
                isPressedX1 = false;
            }


            //Outtake Specimen
            if (gamepad1.b)
                isPressedB1 = true;
            if (!gamepad1.b && isPressedB1) {
                if (outtakeFSM.currentStateOutake != outtakeSpecimen && outtakeFSM.currentStateOutake != outtakeSpecimenHang) {
                    outtakeFSM.setState(outtakeSpecimen);
                    outtakeFSM.executeCurrentState();
                } else if (outtakeFSM.currentStateOutake == outtakeSpecimen) {
                    outtakeFSM.setState(outtakeSpecimenHang);
                    outtakeFSM.executeCurrentState();
                } else if (outtakeFSM.currentStateOutake == outtakeSpecimenHang) {
                    outakeSampleServoPosition = servoextended;
                    goToPickUp = true;
                    startingTimer5 = System.currentTimeMillis();
                } else telemetryOhNo = true;
                isPressedB1 = false;
            }
            if (goToPickUp && startingTimer5 + afterSpecimenOpenTime < System.currentTimeMillis()) {
                outtakeFSM.setState(outtakeSamplePickUp);
                outtakeFSM.executeCurrentState();
                goToPickUp = false;
            }


            //Intake positions
            if (gamepad2.dpad_left) {
                intakeTargetPos = 766;
                gravityAdder = 1;
            } // 4/4
            if (gamepad2.dpad_down) {
                intakeTargetPos = 574;
                gravityAdder = 1;
            } // 3/4
            if (gamepad2.dpad_right){
                intakeTargetPos = 383; // 2/4
                gravityAdder = 1;
            }
            if(gamepad2.dpad_up) {
                intakeTargetPos = 191;  // 1/4
                gravityAdder = 1;
            }
            if(gamepad2.left_bumper)
                intakeTargetPos = 0;



            //Outake HM
            if(gamepad1.y){
                isPressedY1 = true;
            }
            if(!gamepad1.y && isPressedY1){
                if(outtakeFSM.currentStateOutake != outakeHM) {
                    outtakeFSM.setState(outakeHM);
                    outtakeFSM.executeCurrentState();
                }
                else if(outtakeFSM.currentStateOutake == outakeHM){
                    outakeSampleServoPosition= servoextended;
                    startingTimer2=System.currentTimeMillis();
                    wasOutputHM = true;
                }
                isPressedY1 = false;
            }
            if(System.currentTimeMillis() > startingTimer2 + 800 && wasOutputHM){
                outtakeFSM.setState(outtakeSamplePickUp);
                outtakeFSM.executeCurrentState();
                wasOutputHM = false;
            }


            ///END OF STATES CODE

            //OutakeIncrement
            if(gamepad1.dpad_up)
                outakeTargetPosAdder += 2;
            if(gamepad1.dpad_down)
                outakeTargetPosAdder -= 2;



            //Intake Increment
            if (gamepad2.right_trigger > 0.4 && intakeTargetPos <= 270)
                intakeTargetPosAdder += 2;
            if (gamepad2.left_trigger > 0.4 && intakeTargetPos >= 0)
                intakeTargetPosAdder -= 2;


            //Outputing Samples if Nedded
            if(intakeServoPower!= -0.7)
                rememberPosOfServoOut = intakeServoPower;
            if(Toggle.outputtoggle(gamepad1.right_bumper || wasBadSample)!=0){
                intakeServoPower = -0.7;
                wasBadSample = false;
            }
            else intakeServoPower = rememberPosOfServoOut;


            //GAMEPAD2 RELEASE TRIGGER
            if(gamepad2.y)
                isPressedY2 = true;
            if (!gamepad2.y && isPressedY2) {
                if (outakeSampleServoPosition != servoextended)
                    outakeSampleServoPosition = servoextended;
                else if (outakeSampleServoPosition == servoextended)
                    outakeSampleServoPosition = 10;
                else telemetryOhNo = true;
                isPressedY2 = false;
            }



            //Intake target position
            if (intakeinput < 0)
                intakeTargetPos += 15;
            if (intakeinput > 0)
                intakeTargetPos -= 15;



            //PID STUFF
            double intakeMotorPower = 0;
            intakeMotorPower = intakeControlMotor.PIDControl(intakeTargetPos+intakeActualZero+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
            double outakeMotorPower;
            outakeMotorPower = outakeControlMotor.PIDControlUppy(outakeTargetPos-outakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
            outakeMotorPower *= PIDincrement;



            //calculating nedded power by method 1
            frontRightPowerCat = (pivot - vertical - horizontal);
            backRightPowerCat = (pivot - vertical + horizontal);
            frontLeftPowerCat = (pivot + vertical - horizontal);
            backLeftPowerCat = (pivot + vertical + horizontal);



            //TELEMETRY

            //telemetry.addData("intakerotatey",intakeRotateServo.getPosition());
            //telemetry.addData("intakerotateygoalposition",intakeRotateServoPower);
            //telemetry, again
            /*telemetry.addData("IMU angular velocity",imu.getRobotAngularVelocity(AngleUnit.DEGREES));
            telemetry.addData("getRobotOrientationAsQuaternion",imu.getRobotOrientationAsQuaternion());
            telemetry.addData("getRobotYawPitchRollAngles",imu.getRobotYawPitchRollAngles());//*/
            telemetry.addData("outake Error", outakeControlMotor.getLastError());
            telemetry.addData("curent Pos RIGHT", outakeRightMotor.getCurrentPosition());
            telemetry.addData("target Pos", outakeTargetPos);
            telemetry.addData("powah", outakeMotorPower);
            telemetry.addData("OutakeAdder", outakeTargetPosAdder);
            telemetry.addData("spinytimerstart", spinyOutputToggle);//*/
            telemetry.addData("intakeCurentPOs", intakeMotor.getCurrentPosition());
            telemetry.addData("intakeTargetPos", intakeTargetPos);
            telemetry.addData("intakepower", intakeMotorPower);//*/
            //telemetry.addData("servo POs", tester.getPosition());
            //telemetry.addData("y", pos.y);
            //telemetry.addData("x", pos.x);
            //telemetry.addData("rotation/orientation", pos.h);
            //telemetry.addData("outakeArmServoCurrentPosition",outakeArmServo.getPosition());
            //telemetry.addData("outakeArmServo",outakeArmServoPosition);
            //telemetry.addData("outakeSampleServoCurrentPosition",outakeSampleServo.getPosition());
            //telemetry.addData("outakeSampleServo",outakeSampleServoPosition);


            //Automatic slow down
            if (    outtakeFSM.currentStateOutake == outtakeBasket
                    //|| outtakeFSM.currentStateOutake == outtakeSpecimen
                    //|| outtakeFSM.currentStateOutake == outtakeSpecimenHang
                    || intakeFSM.currentStateIntake  == intakeExtended) {
                frontLeftPowerCat /= slowydowny;
                backRightPowerCat /= slowydowny;
                frontRightPowerCat /= slowydowny;
                backLeftPowerCat /= slowydowny;
            }//*/


            //Manual Slowdown Function
            if(slowdown){
                frontLeftPowerCat /= slowyDownyManal;
                backRightPowerCat /= slowyDownyManal;
                frontRightPowerCat /= slowyDownyManal;
                backLeftPowerCat /= slowyDownyManal;
            }




            // set motor power
            frontLeftMotor.setPower(frontLeftPowerCat);
            backLeftMotor.setPower(backLeftPowerCat);
            frontRightMotor.setPower(frontRightPowerCat);
            backRightMotor.setPower(backRightPowerCat);
            intakeMotor.setPower(intakeMotorPower);
            outakeRightMotor.setPower(outakeMotorPower);
            outakeLeftMotor.setPower(outakeMotorPower);//*/
            servo1.setPower(-intakeServoPower);
            servo2.setPower(intakeServoPower);


            //Set servo Positions
            intakeRotateServo.setPosition((intakeRotateServoPosition+gravityAdder) / 360);
            outakeArmServo.setPosition(outakeArmServoPosition / 360);
            outakeSampleServo.setPosition(outakeSampleServoPosition / 360);


            telemetry.addData("IntakeFsm",stateStringIntake);
            telemetry.addData("OutakeFsm",stateStringOutake);
            if(telemetryOhNo)
                telemetry.addData("OH NOOOO",true);
            updateTelemetry(telemetry);
        }
        //multiRunnable.stopRunning();
    }

}