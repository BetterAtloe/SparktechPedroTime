package pedroPathing.Autos;

import static pedroPathing.newOld.PositionStorage.PIDincrement;
import static pedroPathing.newOld.PositionStorage.autoTimer;
import static pedroPathing.newOld.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.newOld.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.newOld.PositionStorage.intakeTargetPos;
import static pedroPathing.newOld.PositionStorage.intakeTargetPosAdder;
import static pedroPathing.newOld.PositionStorage.outakeSampleRetracted;
import static pedroPathing.newOld.PositionStorage.outakeSampleServoPosition;
import static pedroPathing.newOld.PositionStorage.outakeTargetPos;
import static pedroPathing.newOld.PositionStorage.outakeTargetPosAdder;
import static pedroPathing.newOld.PositionStorage.servoextended;
import static pedroPathing.newOld.PositionStorage.stopMulthiread;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import pedroPathing.AutoPIDS.ControlMotor;
import pedroPathing.States.IntakeFSM;
import pedroPathing.States.IntakeStateExtendedHM;
import pedroPathing.States.IntakeStateExtendedRo2v2Auto;
import pedroPathing.States.IntakeStateRetractedRo2;
import pedroPathing.States.IntakeStateWallPURetraction;
import pedroPathing.States.OutakeHMandWallPU;
import pedroPathing.States.OuttakeFSM;
import pedroPathing.States.OuttakeSpecimenHang;
import pedroPathing.States.OuttakeSpecimenHangAuto;
import pedroPathing.States.OuttakeStateBasket;
import pedroPathing.States.OuttakeStateSamplePickUp;
import pedroPathing.States.OuttakeStateSpecimen;
import pedroPathing.States.OuttakeStateSpecimenAuto;
import pedroPathing.States.OuttakeStateStandbyDownWithSample;
import pedroPathing.States.OuttakeStateStandbyWithSampleUp;
import pedroPathing.States.OuttakeStateTranfer;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "AutoTestPositions", group = "Examples")
public class AutoTestPositions extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


    // Initialize Outtake states
    OuttakeStateSpecimenAuto outtakeSpecimenAuto = new OuttakeStateSpecimenAuto();
    OuttakeStateSpecimen outtakeSpecimen = new OuttakeStateSpecimen();
    OuttakeSpecimenHang outtakeSpecimenHang = new OuttakeSpecimenHang();
    OuttakeSpecimenHangAuto outtakeSpecimenHangAuto = new OuttakeSpecimenHangAuto();
    OuttakeStateBasket outtakeBasket = new OuttakeStateBasket();
    OuttakeStateSamplePickUp outtakeSamplePickUp = new OuttakeStateSamplePickUp();
    OuttakeStateStandbyDownWithSample outtakeStandbyDown = new OuttakeStateStandbyDownWithSample();
    OuttakeStateTranfer outtakeStateTranfer = new OuttakeStateTranfer();
    OuttakeStateStandbyWithSampleUp outtakeStateStandbyWithSampleUp = new OuttakeStateStandbyWithSampleUp();
    OutakeHMandWallPU outakeHMandWallPU = new OutakeHMandWallPU();

    // Initialize Intake states
    IntakeStateRetractedRo2 intakeRetractedRo2 = new IntakeStateRetractedRo2();
    IntakeStateExtendedRo2v2Auto intakeExtendedRo2v2Auto = new IntakeStateExtendedRo2v2Auto();
    IntakeStateExtendedHM intakeExtendedRo2v2HM = new IntakeStateExtendedHM();
    IntakeStateWallPURetraction intakeStateWallPURetraction = new IntakeStateWallPURetraction();

    // Create the Outtake FSM with the initial state
    OuttakeFSM outtakeFSM = new OuttakeFSM(outtakeStateStandbyWithSampleUp);

    // Create the Intake FSM with the initial state
    IntakeFSM intakeFSM = new IntakeFSM(intakeStateWallPURetraction);














    /**                         Our Paths!                          */
    private int pathState;

    private final Pose startPose = new Pose(-10, 70, Math.toRadians(180)); //start
    private final Pose FirstSampleScorePose = new Pose(1, 85, Math.toRadians(90)); //line 1
    private final Pose SecondSamplePickUP=new Pose(1,87,Math.toRadians(180)); //line 2 //old 124
    private final Pose SecondSampleScore=new Pose(5,78,Math.toRadians(145)); //line 3
    private final Pose ThirdSamplePickUp =new Pose(27,82,Math.toRadians(180)); //line 4
    private final Pose ThirdSampleScore=new Pose(27,78,Math.toRadians(165)); //line 5
    private final Pose FourthSamplePickUp=new Pose(27.3,82,Math.toRadians(200)); //line 6
    private final Pose FourthSampleScore =new Pose(27,78,Math.toRadians(165)); //line 7
    private final Pose sample2ObservationZonePose =new Pose(18,20,Math.toRadians(0)); //line 8
    // private final Pose sample3LeftPose =new Pose(62,12,Math.toRadians(0)); //line 9
    // private final Pose sample3MovePose=new Pose(62,6,Math.toRadians(0)); //line 10
    // private final Pose sample3ObservationZonePose =new Pose(15,6,Math.toRadians(0)); //line 11//
    private final Pose getSpecimenPose=new Pose(0.8,20,Math.toRadians(0));// line 12
    private final Pose beforeGetSpecimenPose=new Pose(5,20,Math.toRadians(0));// line 12
    private final Pose ScoreSpecimenPose=new Pose(25,60,Math.toRadians(0)); //all the same, line 13
    private final Pose specimen1Score = new Pose(25,56,Math.toRadians(0)); //all the same, line 13
    private final Pose specimen2Score = new Pose(25,58,Math.toRadians(0)); //all the same, line 13
    private final Pose specimen3Score = new Pose(25,62,Math.toRadians(0)); //all the same, line 13
    private final Pose pocketSpecimenPose = ScoreSpecimenPose;
    //private final Pose parkingPose=new Pose(5,5,Math.toRadians(0)); //parking
    private final Pose parkingPose=new Pose(27,82,Math.toRadians(180)); //parking

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;

    ExecutorService executorService = Executors.newFixedThreadPool(2);



    private Path forward,parking;
    private PathChain goToPickUpSecondSample,ScoreSecondSample,goToPickUpThirdSample;

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
        forward = new Path(new BezierLine(new Point(startPose), new Point(FirstSampleScorePose)));
        forward.setLinearHeadingInterpolation(startPose.getHeading(), FirstSampleScorePose.getHeading());

        goToPickUpSecondSample=follower.pathBuilder()
                //goes from bar to an intermediete pose (submersible corner)
                .addPath(new BezierLine(new Point(FirstSampleScorePose), new Point(SecondSamplePickUP)))
                .setLinearHeadingInterpolation(FirstSampleScorePose.getHeading(), SecondSamplePickUP.getHeading())
                .build();
        ScoreSecondSample=follower.pathBuilder()
                //goes from bar to an intermediete pose (submersible corner)
                .addPath(new BezierLine(new Point(SecondSamplePickUP), new Point(SecondSampleScore)))
                .setLinearHeadingInterpolation(SecondSamplePickUP.getHeading(), SecondSampleScore.getHeading())
                .build();
        goToPickUpThirdSample=follower.pathBuilder()
                //goes from bar to an intermediete pose (submersible corner)
                .addPath(new BezierLine(new Point(SecondSampleScore), new Point(ThirdSamplePickUp)))
                .setLinearHeadingInterpolation(SecondSampleScore.getHeading(), ThirdSamplePickUp.getHeading())
                .build();

        //Go from bar after puttin Specimen 4 to parking and end path
        parking = new Path(new BezierLine(new Point(specimen3Score), new Point(parkingPose)));
        parking.setLinearHeadingInterpolation(specimen3Score.getHeading(), parkingPose.getHeading());

    }


    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        switch (pathState) {
            case 12:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    outtakeFSM.setState(outtakeBasket);
                    outtakeFSM.executeCurrentState();
                    while(autoTimer + 500 > System.currentTimeMillis()){}
                        follower.followPath(forward,true);
                        setPathState(1);

                }
                break;

            case 1:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    outakeSampleServoPosition = servoextended;
                    outtakeFSM.setState(outtakeStateStandbyWithSampleUp);
                    outtakeFSM.executeCurrentState();
                    follower.followPath(goToPickUpSecondSample,true);
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    while(autoTimer + 1000 > System.currentTimeMillis()){}
                    autoTimer = System.currentTimeMillis();
                    intakeFSM.setState(intakeExtendedRo2v2Auto);
                    intakeFSM.executeCurrentState();
                    while(autoTimer + 1500 > System.currentTimeMillis() && (colors.red <= 0.0012 && colors.blue <= 0.0012)){
                        colors = colorSensor.getNormalizedColors();
                        if(autoTimer + 200 < System.currentTimeMillis()) {
                            intakeTargetPos = 510;
                        }
                        if(autoTimer + 300 < System.currentTimeMillis()) {
                            intakeMotorPickUpPower = 1;
                        }
                    }
                    while(autoTimer + 1000 > System.currentTimeMillis()){}
                    outtakeFSM.setState(outtakeBasket);
                    outtakeFSM.executeCurrentState();
                    while(autoTimer + 1000 > System.currentTimeMillis()){}
                    follower.followPath(ScoreSecondSample, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    //follower.followPath(goToPickUpThirdSample,true);
                    follower.followPath(parking,true);
                    setPathState(-1);
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
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();


        // our stuff
        executorService.execute(new Runnable() {
            @Override
            public void run() {

                stopMulthiread = false;

                double intakeMotorPower = 0;
                double outakeMotorPower = 0;
                DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
                DcMotor intakeSpinMotor = hardwareMap.dcMotor.get("intakespin");
                DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
                DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                intakeControlMotor = new ControlMotor();
                outakeControlMotor = new ControlMotor();
                //servos
                outakeSampleServoPosition = outakeSampleRetracted;
                Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
                Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
                Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");

                NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");

                NormalizedRGBA colors = colorSensor.getNormalizedColors();

                stopMulthiread = false;
                while(!stopMulthiread){
                    intakeMotorPower = intakeControlMotor.PIDControl(intakeTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
                    outakeMotorPower = outakeControlMotor.PIDControlUppy(outakeTargetPos-outakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
                    outakeMotorPower *= PIDincrement;
                    /*
                    intakeMotor.setPower(intakeMotorPower);
                    outakeRightMotor.setPower(outakeMotorPower);
                    outakeLeftMotor.setPower(outakeMotorPower);
                    intakeSpinMotor.setPower(intakeMotorPickUpPower);


                    //Set servo Positions
                    intakeRotateServo.setPosition((intakeRotateServoPosition) / 360);
                    outakeArmServo.setPosition(outakeArmServoPosition / 360);
                    outakeSampleServo.setPosition(outakeSampleServoPosition / 360);

                    /*telemetry.addData("frontLeftPowerCat",frontLeftPowerCat);
                    telemetry.addData("backLeftPowerCat",backLeftPowerCat);
                    telemetry.addData("frontRightPowerCat",frontRightPowerCat);
                    telemetry.addData("backRightPowerCat",backRightPowerCat);

                    telemetry.addLine("This is Motor "+Thread.currentThread().getId());
                    updateTelemetry(telemetry);
                       //*/
                }
            }
        });
        //*/
        //end of our stuff
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        //tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        //follower.telemetryDebug(tel);
        Drawing.drawDebug(follower);

        // Feedback to Driver Hub
        tel.addData("path state", pathState);
        tel.addData("x", follower.getPose().getX());
        tel.addData("y", follower.getPose().getY());
        tel.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        tel.addData("intakerotate",intakeRotateServoPosition);
        tel.update();
    }



    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

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
        super.stop();
        executorService.shutdownNow();
        stopMulthiread = true;
    }




}