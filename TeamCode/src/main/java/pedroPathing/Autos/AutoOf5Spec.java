package pedroPathing.Autos;

import static pedroPathing.ClassWithStates.ColorCompare;
import static pedroPathing.ClassWithStates.autoOuttakeWallPickUpNew;
import static pedroPathing.ClassWithStates.colorList;
import static pedroPathing.ClassWithStates.colorSensorOutty;
import static pedroPathing.ClassWithStates.currentStateOfSampleInIntake;
import static pedroPathing.ClassWithStates.currentTeam;
import static pedroPathing.ClassWithStates.initStates;
import static pedroPathing.ClassWithStates.intakeCabinDownCollecting;
import static pedroPathing.ClassWithStates.intakeCabinFullInBot;
import static pedroPathing.ClassWithStates.intakeCabinFullInBotOutputting;
import static pedroPathing.ClassWithStates.intakeExtended4out4;
import static pedroPathing.ClassWithStates.intakeRetracted;
import static pedroPathing.ClassWithStates.outtakeSpecimenHang;
import static pedroPathing.ClassWithStates.outtakeStandByWithoutExtensions;
import static pedroPathing.OrganizedPositionStorage.autoTimer;
import static pedroPathing.OrganizedPositionStorage.intakeExtendMotorTargetPos;
import static pedroPathing.OrganizedPositionStorage.intakeGravitySubtractor;
import static pedroPathing.OrganizedPositionStorage.intakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.intakeSpinMotorPow;
import static pedroPathing.OrganizedPositionStorage.intakeTargetPosAdder;
import static pedroPathing.OrganizedPositionStorage.isRobotInAuto;
import static pedroPathing.OrganizedPositionStorage.isYellowSampleNotGood;
import static pedroPathing.OrganizedPositionStorage.needsToExtraExtend;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoExtendedPos;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoExtraExtendedPos;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoRetractedPos;
import static pedroPathing.OrganizedPositionStorage.outtakeExtendMotorTargetPos;
import static pedroPathing.OrganizedPositionStorage.outtakeIsInNeedToExtraExtendClawTimer;
import static pedroPathing.OrganizedPositionStorage.outtakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.resetStuff;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.AutoPIDS.ControlMotor;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "AutoOf5Spec", group = "Examples")
public class AutoOf5Spec extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    final float[] hsvValues = new float[3];
    private DcMotor intakeMotor;
    private DcMotor outakeLeftMotor;
    private DcMotor outakeRightMotor;
    private DcMotor intakeSpinMotor;
    private Servo intakeRotateServo;
    private Servo outakeArmServo;
    private Servo outakeSampleServo;
    private NormalizedColorSensor colorSensor;

    /**                         Our Paths!                          */
    private int pathState;

    private final Pose startPose = new Pose(-10, 70, Math.toRadians(90)); //start
    //scoring bar positions
    private final float scoringBarX = -2f;
    private final float scoringBarY = 41.3f;
    private final Pose scoringBarPosePreloadSpecimen = new Pose(scoringBarX, scoringBarY, Math.toRadians(90)); //start
    private final Pose scoringBarPoseFirstSpecimen = new Pose(scoringBarX-2, scoringBarY+1, Math.toRadians(90)); //start
    private final Pose scoringBarPoseSecondSpecimen = new Pose(scoringBarX-3, scoringBarY+1, Math.toRadians(90)); //start
    private final Pose scoringBarPoseThirdSpecimen = new Pose(scoringBarX-4, scoringBarY+1, Math.toRadians(90)); //start
    private final Pose scoringBarPoseFourthSpecimen = new Pose(scoringBarX-4, scoringBarY+1, Math.toRadians(90)); //start
    //private final Pose scoringBarPoseFifthSpecimen = new Pose(-5.4, 44 + globalSpecimenYOffset, Math.toRadians(90)); //start

    private final float wallPickUpX = -42f;
    private final float wallPickUpY = 71f;
    private final float wallAdderY = 2f;

    //specimen pick up positions
    private final Pose firstSpecimenPickUpPose = new Pose(wallPickUpX, wallPickUpY, Math.toRadians(90)); //start
    private final Pose secondSpecimenPickUpPose = new Pose(wallPickUpX, wallPickUpY+wallAdderY, Math.toRadians(90)); //start
    private final Pose thirdSpecimenPickUpPose = new Pose(wallPickUpX, wallPickUpY+wallAdderY+0.2, Math.toRadians(90)); //start
    private final Pose fourthSpecimenPickUpPose = new Pose(wallPickUpX, wallPickUpY+wallAdderY+0.4, Math.toRadians(90)); //start

    // ----------------------------------------------- SAMPLE POSES ----------------------------------------------- \\

    private final Pose firstSamplePickUpPos = new Pose(-58.5,60,Math.toRadians(112)); //start
    private final Pose secondSamplePickUpPos = new Pose(-59, 61, Math.toRadians(100)); //start
    private final Pose thirdSamplePickUpPos = new Pose(-51, 64, Math.toRadians(64)); //start

    //PARK
    private final Pose parkingPose=new Pose(-55,70 - 0.5,Math.toRadians(90)); //parking
    double intakeMotorPower=0;
    double outakeMotorPower=0;


    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    private Path startPath,pickUpFirst,pickUpSecond,pickUpThird,pickUpFourth,pickUpFifth,scoreFirst,scoreSecond,scoreThird,scoreFourth,scoreFifth,parking;
    private PathChain goToPickUpFirstSample,goToPickUpSecondSample,goToPickUpThirdSample;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        startPath = new Path(new BezierLine(new Point(startPose), new Point(scoringBarPosePreloadSpecimen)));
        startPath.setLinearHeadingInterpolation(startPose.getHeading(), scoringBarPosePreloadSpecimen.getHeading());

        /*goToPickUpFirstSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoringBarPosePreloadSpecimen), new Point(firstSamplePickUpPos)))
                .setLinearHeadingInterpolation(scoringBarPosePreloadSpecimen.getHeading(), firstSamplePickUpPos.getHeading())
                .build();

        goToPickUpSecondSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSamplePickUpPos), new Point(secondSamplePickUpPos)))
                .setLinearHeadingInterpolation(firstSamplePickUpPos.getHeading(), secondSamplePickUpPos.getHeading())
                .build();

        goToPickUpThirdSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSamplePickUpPos), new Point(thirdSamplePickUpPos)))
                .setLinearHeadingInterpolation(secondSamplePickUpPos.getHeading(), thirdSamplePickUpPos.getHeading())
                .build();
*/  // 3 2 1 logic
        goToPickUpFirstSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoringBarPosePreloadSpecimen), new Point(thirdSamplePickUpPos)))
                .setLinearHeadingInterpolation(scoringBarPosePreloadSpecimen.getHeading(), thirdSamplePickUpPos.getHeading())
                .build();

        goToPickUpSecondSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSamplePickUpPos), new Point(secondSamplePickUpPos)))
                .setLinearHeadingInterpolation(thirdSamplePickUpPos.getHeading(), secondSamplePickUpPos.getHeading())
                .build();

        goToPickUpThirdSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSamplePickUpPos), new Point(firstSamplePickUpPos)))
                .setLinearHeadingInterpolation(secondSamplePickUpPos.getHeading(), firstSamplePickUpPos.getHeading())
                .build();

        //first spec //changed logic to for 3 2 1 logic
        pickUpFirst = new Path(new BezierLine(new Point(firstSamplePickUpPos), new Point(firstSpecimenPickUpPose)));
        pickUpFirst.setLinearHeadingInterpolation(firstSamplePickUpPos.getHeading(), firstSpecimenPickUpPose.getHeading());

        scoreFirst = new Path(new BezierLine(new Point(firstSpecimenPickUpPose), new Point(scoringBarPoseFirstSpecimen)));
        scoreFirst.setLinearHeadingInterpolation(firstSpecimenPickUpPose.getHeading(), scoringBarPoseFirstSpecimen.getHeading());
        //second spec

        pickUpSecond = new Path(new BezierLine(new Point(scoringBarPoseFirstSpecimen), new Point(secondSpecimenPickUpPose)));
        pickUpSecond.setLinearHeadingInterpolation(scoringBarPoseFirstSpecimen.getHeading(), secondSpecimenPickUpPose.getHeading());

        scoreSecond = new Path(new BezierLine(new Point(secondSpecimenPickUpPose), new Point(scoringBarPoseSecondSpecimen)));
        scoreSecond.setLinearHeadingInterpolation(secondSpecimenPickUpPose.getHeading(), scoringBarPoseSecondSpecimen.getHeading());


        //third spec

        pickUpThird = new Path(new BezierLine(new Point(scoringBarPoseSecondSpecimen), new Point(thirdSpecimenPickUpPose)));
        pickUpThird.setLinearHeadingInterpolation(scoringBarPoseSecondSpecimen.getHeading(), thirdSpecimenPickUpPose.getHeading());

        scoreThird = new Path(new BezierLine(new Point(thirdSpecimenPickUpPose), new Point(scoringBarPoseThirdSpecimen)));
        scoreThird.setLinearHeadingInterpolation(thirdSpecimenPickUpPose.getHeading(), scoringBarPoseThirdSpecimen.getHeading());


        //fourth spec

        pickUpFourth = new Path(new BezierLine(new Point(scoringBarPoseThirdSpecimen), new Point(fourthSpecimenPickUpPose)));
        pickUpFourth.setLinearHeadingInterpolation(scoringBarPoseThirdSpecimen.getHeading(), fourthSpecimenPickUpPose.getHeading());

        scoreFourth = new Path(new BezierLine(new Point(fourthSpecimenPickUpPose), new Point(scoringBarPoseFourthSpecimen)));
        scoreFourth.setLinearHeadingInterpolation(fourthSpecimenPickUpPose.getHeading(), scoringBarPoseFourthSpecimen.getHeading());


        //parking
        parking = new Path(new BezierLine(new Point(scoringBarPoseFourthSpecimen), new Point(parkingPose)));
        parking.setLinearHeadingInterpolation(scoringBarPoseFourthSpecimen.getHeading(), parkingPose.getHeading());

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    // from 1 - 100 is normal paths
    // from 100+ is scoring paths

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    outtakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(startPath,true);
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(150);
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(goToPickUpFirstSample,true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    intakeCabinDownCollecting();
                    waitWhile(300);
                    intakeExtended4out4();
                    autoTimer = System.currentTimeMillis();
                    while(!(currentStateOfSampleInIntake == colorSensorOutty.correctSample)  && autoTimer + 2000 > System.currentTimeMillis()) robotDoStuff();
                    intakeRetracted();
                    intakeCabinFullInBot();
                    waitWhile(300);



                    follower.followPath(goToPickUpSecondSample,true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {

                    intakeCabinFullInBotOutputting();
                    while(currentStateOfSampleInIntake == colorSensorOutty.correctSample) robotDoStuff();

                    waitWhile(300);
                    //output first picked upaka 3rd sample counting from subermsible



                    //pick up 2nd sample
                    autoTimer = System.currentTimeMillis();
                    intakeCabinDownCollecting();
                    waitWhile(300);
                    intakeExtended4out4();

                    autoTimer = System.currentTimeMillis();
                    while(!(currentStateOfSampleInIntake == colorSensorOutty.correctSample)  && autoTimer + 2000 > System.currentTimeMillis()) robotDoStuff();
                    intakeRetracted();
                    intakeCabinFullInBot();
                    waitWhile(300);
                    /*
                    intakeCabinFullInBotOutputting();
                    while(currentStateOfSampleInIntake == colorSensorOutty.correctSample) robotDoStuff();
                    */


                    follower.followPath(goToPickUpThirdSample,true);
                    setPathState(105);
                }
                break;

            //last sample and first spec prep
            case 105:
                if(!follower.isBusy()) {
                    //trow out second sample
                    intakeCabinFullInBotOutputting();
                    while(currentStateOfSampleInIntake == colorSensorOutty.correctSample) robotDoStuff();

                    waitWhile(300);


                    //pick up and trow third sample  aka 1st from submersible
                    autoTimer = System.currentTimeMillis();
                    intakeCabinDownCollecting();
                    waitWhile(300);
                    intakeExtended4out4();

                    autoTimer = System.currentTimeMillis();
                    while(!(currentStateOfSampleInIntake == colorSensorOutty.correctSample)  && autoTimer + 2000 > System.currentTimeMillis()) robotDoStuff();
                    intakeRetracted();
                    intakeCabinFullInBot();
                    waitWhile(350);
                    intakeCabinFullInBotOutputting();
                    while(currentStateOfSampleInIntake == colorSensorOutty.correctSample) robotDoStuff();



                    //prep for pick up
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(300);
                    autoOuttakeWallPickUpNew();
                    intakeCabinFullInBot();
                    autoTimer = System.currentTimeMillis();




                    follower.followPath(pickUpFirst,true);
                    setPathState(106);
                }
                break;
            case 106:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    waitWhile(150);
                    outtakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(scoreFirst,true);
                    setPathState(107);
                }
                break;

            //second spec
            case 107:
                if(!follower.isBusy()) {
                    autoOuttakeWallPickUpNew();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(pickUpSecond,true);
                    setPathState(108);
                }
                break;
            case 108:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    waitWhile(150);
                    outtakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(scoreSecond,true);
                    setPathState(109);
                }
                break;

            //third spec
            case 109:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(150);
                    autoOuttakeWallPickUpNew();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(pickUpThird,true);
                    setPathState(110);
                }
                break;
            case 110:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    waitWhile(150);
                    outtakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(scoreThird,true);
                    setPathState(111);
                }
                break;

            //fourth spec
            case 111:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(150);
                    autoOuttakeWallPickUpNew();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(pickUpFourth,true);
                    setPathState(112);
                }
                break;
            case 112:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    waitWhile(150);
                    outtakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(scoreFourth,true);
                    setPathState(113);
                }
                break;


            case 113:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(150);
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(parking,true);
                    setPathState(-1);
                    outtakeStandByWithoutExtensions();
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        resetStuff();
        isRobotInAuto = true;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        setPathState(0);


        //our init
        intakeMotor = hardwareMap.dcMotor.get("intakemotor");
        outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
        intakeSpinMotor = hardwareMap.dcMotor.get("intakespin");


        //declare servos
        intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");


        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");

        isYellowSampleNotGood = true;

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);//*/
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeControlMotor = new ControlMotor();
        outakeControlMotor = new ControlMotor();

        // Set init position
        initStates();
        intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
        outakeArmServo.setPosition(outtakePivotServoPos / 328);
        outakeSampleServo.setPosition(outtakeClawServoPos / 360);
        //end of our stuff
    }


    public void robotDoStuff(){
        //ifs
        if(needsToExtraExtend && outtakeIsInNeedToExtraExtendClawTimer + 400 < System.currentTimeMillis()){
            needsToExtraExtend = false;
            outtakeClawServoPos = outtakeClawServoExtraExtendedPos;
        }
        if (follower.getVelocity().getMagnitude() == 0 && follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
            follower.breakFollowing();
        }


        //color stuff
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        currentStateOfSampleInIntake = ColorCompare(colors,currentTeam,isYellowSampleNotGood);

        //PID Stuff
        intakeMotorPower = intakeControlMotor.PIDControl(intakeExtendMotorTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
        outakeMotorPower = outakeControlMotor.PIDControlUppy(-outtakeExtendMotorTargetPos, outakeLeftMotor.getCurrentPosition());

        //set motor positions
        intakeMotor.setPower(intakeMotorPower);
        outakeRightMotor.setPower(outakeMotorPower);
        outakeLeftMotor.setPower(outakeMotorPower);
        intakeSpinMotor.setPower(intakeSpinMotorPow);
        tel.addData("intakeSpinMotor",intakeSpinMotor.getPower());
        tel.addData("intakeSpinMotorInCode",intakeSpinMotorPow);

        //Set servo Positions
        intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
        outakeArmServo.setPosition(outtakePivotServoPos / 328);
        outakeSampleServo.setPosition(outtakeClawServoPos / 360);

        robotTelemetry();
    }


    public void waitWhile(int timeToWait) {
        long iniTime = System.currentTimeMillis();
        while(iniTime + timeToWait > System.currentTimeMillis()){
            robotDoStuff();
        }
    }


    void robotTelemetry(){
        tel.addData("path state", pathState);
        tel.addData("x", follower.getPose().getX());
        tel.addData("y", follower.getPose().getY());
        tel.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        tel.addData("intakePivotServoPos",intakePivotServoPos);
        tel.addData("outtakeTargetPos",outtakeExtendMotorTargetPos);
        tel.addData("outtakeDirection",outakeMotorPower);
        tel.addData("outtakeCurrenPos",outakeLeftMotor.getCurrentPosition());
        tel.addData("sensor color",currentStateOfSampleInIntake);
        tel.addData("follower is busy",follower.isBusy());
        Drawing.drawDebug(follower);
        tel.update();
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        robotDoStuff();


        // Feedback to Driver Hub
        robotTelemetry();
    }



    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        if ((gamepad2.left_bumper && gamepad2.start) || (gamepad1.left_bumper && gamepad1.start))
            currentTeam = colorList.blue;
        if ((gamepad2.right_bumper && gamepad2.start) || (gamepad1.right_bumper && gamepad1.start))
            currentTeam = colorList.red;
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }




}