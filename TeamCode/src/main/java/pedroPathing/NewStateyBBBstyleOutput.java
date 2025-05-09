package pedroPathing;


import static pedroPathing.OrganizedPositionStorage.*;
import static pedroPathing.ClassWithStates.*;
import pedroPathing.newOld.Toggle;


import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.tests.Config;


@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "BBBNewStatesOutput", group = "Linear OpMode")
public class NewStateyBBBstyleOutput extends LinearOpMode {

    final float[] hsvValues = new float[3];

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    SparkFunOTOS myOtos;
    NormalizedColorSensor colorSensor;
    Queuer queuer = new Queuer();
    private Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {

        OrganizedPositionStorage.resetStuff();

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
        DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
        DcMotor intakeSpinMotor = hardwareMap.dcMotor.get("intakespin");
        myOtos = hardwareMap.get(SparkFunOTOS.class, "SparkFunSensor");


        //declare servos
        Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");


        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);//*/
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSpinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Config.configureOtos(tel, myOtos);


        intakeControlMotor = new ControlMotor();
        outakeControlMotor = new ControlMotor();

        // Set init position
        initStates();
        intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
        outakeArmServo.setPosition(outtakePivotServoPos / 328);
        outakeSampleServo.setPosition(outtakeClawServoPos / 360);



        waitForStart();

        if (isStopRequested()){
            return;
        }

        while (opModeIsActive()) {
            ///gamepad1
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = -gamepad1.right_stick_x;

            ///gamepad2
            double intakeinput = gamepad2.left_stick_y;
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);


            //Selectare Echipa
            if ((gamepad2.left_bumper && gamepad2.start) || (gamepad1.left_bumper && gamepad1.start))
                currentTeam = colorList.blue;
            if ((gamepad2.right_bumper && gamepad2.start) || (gamepad1.right_bumper && gamepad1.start))
                currentTeam = colorList.red;


            //problem with dashboard stick
            if(gamepad2.right_trigger >= 0.4 && gamepad2.left_trigger >= 0.4) reverseGamepad2=true;

            if(reverseGamepad2) pivot = -gamepad1.right_stick_y;



            //selectare mod de control, reversing bot
            if(Toggle.FirsToggle(gamepad1.right_trigger >= 0.4 && gamepad1.left_trigger >= 0.4)){
                vertical = -vertical;
                horizontal = -horizontal;
                pivot = pivot;
            }




            //color compare for intake
            currentStateOfSampleInIntake = ColorCompare(colors,currentTeam,isYellowSampleNotGood);



            ///CONTROLS

            //PICK UP
            if(gamepad1.a) isPressedA1 = true;
            if(!gamepad1.a && isPressedA1){
                if(intakeCabinState == intakeCabinStates.intakeCabinFullInBot && (intakeState==intakeStates.intakeRetracted || intakeState == intakeStates.intakeExtended1out4)){
                    isInPositionToRaiseOuttakeInOrderToEvadeIntake = true;
                    waitingForOuttakeToEvadeIntakeTimer = System.currentTimeMillis();
                    outtakeSpecimenHang();
                }
                else if(!(intakeCabinState == intakeCabinStates.intakeCabinDownCollecting) && !(intakeCabinState == intakeCabinStates.intakeCabinDownOutputting)) {
                    intakeCabinDownCollecting();
                    if(!isInSpecimenState)
                        outtakeTransfer();
                    isAfterIntakeBeenDownColecting = true;
                }
                else{
                    intakeRetracted();
                    intakeCabinTransferPosition();
                    if(!isInSpecimenState) {
                        outtakeTransfer();
                    }
                    else{
                        intakeCabinFullInBot();
                    }
                    isAfterIntakeBeenDownColecting = false;
                }
                isPressedA1 = false;
            }


            if(gamepad2.x) isPressedX2 = true;
            if(!gamepad2.x && isPressedX2){
                outtakeStandByWithoutExtensions();
                isInPositionToRaiseOuttakeInOrderToEvadeIntake = false;
                intakeRetracted();
                intakeCabinTransferPosition();
                isPressedX2 = false;
            }


            //SPECIMEN
            if(gamepad1.b) isPressedB1 = true;
            if(!gamepad1.b && isPressedB1){
                if(!(outtakeState == outtakeStates.outtakeSpecimenHang)) {
                    outtakeClawServoPos = 12;
                    isAfterOuttakeClosedClawAtWallSpecimen = true;
                    outtakeAfterHasClosedClawAtWallSpecimenTimer = System.currentTimeMillis();
                }
                else{
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    outtakeSpecimenAfterScoreTimer = System.currentTimeMillis();
                    isAfterOuttakeScoredSpecimen = true;
                }
                isPressedB1 = false;
            }
            if(isAfterOuttakeClosedClawAtWallSpecimen && outtakeAfterHasClosedClawAtWallSpecimenTimer + 300 < System.currentTimeMillis()){
                intakeRetracted();
                intakeCabinFullInBot();
                outtakeSpecimenHang();
                isAfterOuttakeClosedClawAtWallSpecimen = false;
            }
            if(isAfterOuttakeScoredSpecimen && outtakeSpecimenAfterScoreTimer + 300 < System.currentTimeMillis()){
                outtakeWallPickUpNew();
                isAfterOuttakeScoredSpecimen = false;
            }
            if(outtakeIsInNeedToExtraExtendClaw && outtakeIsInNeedToExtraExtendClawTimer + 650 < System.currentTimeMillis()){
                outtakeIsInNeedToExtraExtendClaw = false;
                outtakeClawServoPos = outtakeClawServoExtraExtendedPos;
            }


            //BASKET SCORING
            if(gamepad1.x) isPressedX1 = true;
            if(!gamepad1.x && isPressedX1){
                if(outtakeState != outtakeStates.outtakeBasket){

                    isAfterOuttakeClawClosedAfterTransfer = true;
                    intakeAfterTransferClosedClawTimer = System.currentTimeMillis();
                    isAtStateOfLettingBasketSampleGo = true;
                }
                isPressedX1 = false;
            }
            if(isAfterOuttakeClawClosedAfterTransfer && intakeAfterTransferClosedClawTimer + 300 < System.currentTimeMillis()){
                intakeRetracted();
                intakeCabinTransferPosition();
                outtakeBasket();
                isAfterOuttakeClawClosedAfterTransfer = false;
            }
            //going down after, quite complicated cuz holding to let sample go
            if(isAtStateOfLettingBasketSampleGo && gamepad1.x){
                outtakeClawServoPos = outtakeClawServoExtendedPos;
                isAfterOuttakeScoredBasketSample = true;
                isAtStateOfLettingBasketSampleGo = false;
                outtakeAfterBasketSampleScoreTimer = System.currentTimeMillis();
            }
            if(!gamepad1.x && isAfterOuttakeScoredBasketSample) {
                outtakePivotServoPos = outtakePivotServoTransferPos;
                if(outtakeAfterBasketSampleScoreTimer + 300 < System.currentTimeMillis()) {
                    outtakeTransfer();
                    isAfterOuttakeScoredBasketSample = false;
                }
            }

            /*WALL PICK UP
            if(gamepad1.y) isPressedY1 = true;
            if(!gamepad1.y && isPressedY1){
                intakeRetracted();
                intakeCabinFullInBot();
                outtakeWallPickUpNew();
                isPressedY1 = false;
            }//*/

            ///SOME STUFF

            //auto retract
            if(currentStateOfSampleInIntake == colorSensorOutty.correctSample && isAfterIntakeBeenDownColecting){
                //retracting the thing WITH POWER
                intakeRetracted();
                intakeCabinTransferPositionWithPower();
                if(!isInSpecimenState) outtakeTransfer();
                isAfterIntakeBeenDownColecting= false;
                intakeSpinMotorMorePowerAfterTakingTimer = System.currentTimeMillis();
                isIntakeSpinMOtorAfterJustTaking  = true;
            }

            //stopping the moving intake
            if(isIntakeSpinMOtorAfterJustTaking && intakeSpinMotorMorePowerAfterTakingTimer + 600 < System.currentTimeMillis()){
                //if needs transfer
                if(!isInSpecimenState){
                    intakeCabinTransferPosition();
                    outtakeCloseClawInTransferTimer = System.currentTimeMillis();
                    isOuttakeInPositionToCloseClawForTransfer = true;
                }

                //if in specimen mode
                else intakeCabinFullInBot();

                //false
                isIntakeSpinMOtorAfterJustTaking = false;
            }

            //closing the claw
            if(isOuttakeInPositionToCloseClawForTransfer && outtakeCloseClawInTransferTimer + 150 < System.currentTimeMillis()){
                outtakeClawServoPos = outtakeClawServoRetractedPos;
                isOuttakeInPositionToCloseClawForTransfer = false;
                outtakeGoToStandByTimer = System.currentTimeMillis();
                isOuttakeInPositionToGoToStandBy = true;
            }

            //wait for the claw to close, then go to standBy
            if(isOuttakeInPositionToGoToStandBy && outtakeGoToStandByTimer + 175 < System.currentTimeMillis()){
                outtakeSpecimenHang();
                isOuttakeInPositionToGoToStandBy = false;
            }

            /*
            if(basketStandbyState == 0 && isIntakeSpinMOtorAfterJustTaking && intakeSpinMotorMorePowerAfterTakingTimer + 100 < System.currentTimeMillis()){

                intakeCabinTransferPosition();
                if(!isInSpecimenState) {
                    outtakeTransfer();
                    basketStandbyState++;
                }
                outtakeClawServoPos = outtakeClawServoExtendedPos;

            }
            if(basketStandbyState == 1 && isIntakeSpinMOtorAfterJustTaking && intakeSpinMotorMorePowerAfterTakingTimer + 500 < System.currentTimeMillis()) {
                outtakeClawServoPos = outtakeClawServoRetractedPos;
                basketStandbyState++;
            }
            if(basketStandbyState == 2 && isIntakeSpinMOtorAfterJustTaking && intakeSpinMotorMorePowerAfterTakingTimer + 1000 < System.currentTimeMillis()) {
                //outtakeClawServoPos = outtakeClawServoRetractedPos;
                basketStandbyState++;
                outtakeSpecimenHang();
                isIntakeSpinMOtorAfterJustTaking = false;
                basketStandbyState = 0;
                //outtakeExtendMotorTargetPos = outtakeMotorStandByPos;

            }//*/


            //auto eject
            if(currentStateOfSampleInIntake == colorSensorOutty.wrongSample){
                intakeCabinDownOutputting();
                isIntakeOutputting = true;
                intakeOutputtingTimer = System.currentTimeMillis();
            }
            if(isIntakeOutputting && intakeOutputtingTimer + 300 < System.currentTimeMillis()){
                intakeCabinDownCollecting();
                isIntakeOutputting = false;
            }



            //manual eject
            if(gamepad1.left_bumper){
                intakeSpinMotorPow = -0.75;
                isIntakeOutputtingManual = true;
                intakeOutputtingTimerManual = System.currentTimeMillis();
            }
            if(isIntakeOutputtingManual && intakeOutputtingTimerManual + 300 < System.currentTimeMillis()){
                if(intakeCabinState == intakeCabinStates.intakeCabinDownCollecting) intakeSpinMotorPow = 1;
                else if(intakeCabinState == intakeCabinStates.intakeCabinDownOutputting) intakeSpinMotorPow = -0.75;
                else intakeSpinMotorPow = 0;
                isIntakeOutputtingManual = false;
            }


            //chosing intake positions
            //Intake positions
            if (gamepad1.dpad_left)  intakeExtended4out4();
            if (gamepad1.dpad_down)  intakeExtended3out4();
            if (gamepad1.dpad_right) intakeExtended2out4();
            if(gamepad1.dpad_up)     intakeExtended1out4();
            if(gamepad2.left_bumper) intakeRetracted();


            //getting to specimen pick up pos without messing cables
            if(isInNeedToGoToSpecimenTransferPos && outakeLeftMotor.getCurrentPosition()<-500){
                outtakePivotServoPos = outtakePivotServoWallPickupPos;
                isInNeedToGoToSpecimenTransferPos = false;
            }

            //getting out of specimen pick up pos without messing cables
            if(isOuttakeInPositionToGoDown){
                if(outtakeState == outtakeStates.outtakeBasket){
                    isOuttakeInPositionToGoDown = false;
                    outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
                }
                else if(beforeOuttakeGoDownTimer + 400 < System.currentTimeMillis()) {
                    outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
                    isOuttakeInPositionToGoDown = false;
                }
            }

            if(isInPositionToRaiseOuttakeInOrderToEvadeIntake && waitingForOuttakeToEvadeIntakeTimer + 150 < System.currentTimeMillis()){
                intakeCabinDownCollecting();
                if(!isInSpecimenState)
                    outtakeTransfer();
                isAfterIntakeBeenDownColecting = true;
                isInPositionToRaiseOuttakeInOrderToEvadeIntake = false;
            }





            if(!(gamepad2.b || gamepad1.y)){
                isTimeToRefreshOutptingTime = true;
                if(intakeCabinState == intakeCabinStates.intakeCabinFullInBotOutputting){
                    intakeCabinFullInBot();
                    outtakeSpecimenHang();
                }

            } else {
                if (isTimeToRefreshOutptingTime) {
                    isTimeToRefreshOutptingTime = false;
                    timeSinceStartedMovingForTruBotOutput = (long) (intakePivotServoPos - 15) * 4 + System.currentTimeMillis();
                    intakeCabinFullInBot();
                    outtakeSpecimenHang();
                } else if (System.currentTimeMillis() > timeSinceStartedMovingForTruBotOutput) {
                    intakeCabinFullInBotOutputting();
                    outtakeSpecimenHang();
                }
            }


            //selecting specimen state or not
            if(gamepad2.a) isPressedA2 = true;
            if(!gamepad2.a && isPressedA2){
                isInSpecimenState = !isInSpecimenState;
                isPressedA2 = false;
            }



            //PIDs
            PIDincrement=1;
            double intakeExtendMotorPow;
            intakeExtendMotorPow = intakeControlMotor.PIDControl(intakeExtendMotorTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
            double outtakeExtendMotorPow;
            outtakeExtendMotorPow = outakeControlMotor.PIDControlUppy(-outtakeExtendMotorTargetPos-outtakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
            outtakeExtendMotorPow *= PIDincrement;




            chassisFrontRightPow = (pivot - vertical - horizontal);
            chassisBackRightPow = (pivot - vertical + horizontal);
            chassisFrontLeftPow = (pivot + vertical - horizontal);
            chassisBackLeftPow = (pivot + vertical + horizontal);


            //slowdown
            double slowyDownyManal = 2.5;
            double slowyDownyAuto = 1.4;

            //manual slowdown
            if(gamepad1.right_bumper){
                chassisFrontLeftPow /= slowyDownyManal;
                chassisBackRightPow /= slowyDownyManal;
                chassisFrontRightPow /= slowyDownyManal;
                chassisBackLeftPow /= slowyDownyManal;
            }
            //auto slowdown
            else if(outtakeState == outtakeStates.outtakeBasket
                    || outtakeState == outtakeStates.outtakeWallPickUpNew
                    //|| outtakeState == outtakeStates.outtakeSpecimenHang
            ){
                chassisFrontLeftPow /= slowyDownyAuto;
                chassisBackRightPow /= slowyDownyAuto;
                chassisFrontRightPow /= slowyDownyAuto;
                chassisBackLeftPow /= slowyDownyAuto;
            }

            // Toggle Claw on Y2

            if(gamepad2.y){
                isPressedY2 = true;
            }
            if(!gamepad2.y && isPressedY2){
                if(outtakeClawServoPos == outtakeClawServoRetractedPos){
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                } else {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                }
                isPressedY2 = false;
            }



            // set motor power
            frontLeftMotor.setPower(chassisFrontLeftPow*0.8);
            backLeftMotor.setPower(chassisBackLeftPow*0.8);
            frontRightMotor.setPower(chassisFrontRightPow*0.8);
            backRightMotor.setPower(chassisBackRightPow*0.8);
            intakeMotor.setPower(intakeExtendMotorPow);
            outakeRightMotor.setPower(outtakeExtendMotorPow);
            outakeLeftMotor.setPower(outtakeExtendMotorPow);
            intakeSpinMotor.setPower(intakeSpinMotorPow);


            //Set servo Positions
            intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
            outakeArmServo.setPosition(outtakePivotServoPos / 328);
            outakeSampleServo.setPosition(outtakeClawServoPos / 360);


            tel.addData("intakeSliderState",intakeState);
            tel.addData("intakeCabinState",intakeCabinState);
            tel.addData("outtakeState",outtakeState);
            tel.addData("color stuff",currentStateOfSampleInIntake);
            tel.addData("outakeArmServoPOS GO TO", outtakePivotServoPos);
            tel.addData("outakeSamplePOS GO TO ", outtakeClawServoPos);
            tel.addData("intakeRotateServoPosition", intakePivotServoPos);
            tel.addData("intakeExtendMotorPow",intakeExtendMotorPow);
            tel.addData("outakeMotorPow",outtakeExtendMotorPow);
            tel.addData("outtakeTargetPos",outtakeExtendMotorTargetPos);
            tel.addData("outtake current pos",outakeLeftMotor.getCurrentPosition());

            tel.update();
        }

    }

}