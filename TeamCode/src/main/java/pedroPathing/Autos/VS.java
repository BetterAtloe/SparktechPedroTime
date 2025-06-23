package pedroPathing.Autos;

import static pedroPathing.ClassWithStates.*;
import static pedroPathing.OrganizedPositionStorage.*;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import pedroPathing.AutoPIDS.ControlMotor;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "Vision Strafe", group = "Examples")
public class VS extends OpMode {
    private FtcDashboard dashboard;
    private Telemetry dashboardTelemetry;

    /// SAMPLE DETECTION VALUES
    private final ColorRange FTC_YELLOW = new ColorRange(
            ColorSpace.RGB,
            new Scalar( 220, 220,   0),  //yellow
            new Scalar(255, 255, 255)
    );
    private final ColorRange FTC_RED = new ColorRange(
            ColorSpace.RGB,
            new Scalar( 215, 0,  0),  //red
            new Scalar(255, 219, 180)
    );
    private final ColorRange FTC_BLUE = new ColorRange(
            ColorSpace.RGB,
            new Scalar( 16,   0, 215), //blue
            new Scalar(255, 127, 255)
    );
    /// NOT SETTINGS
    private ColorBlobLocatorProcessor colorLocatorBlue;
    private ColorBlobLocatorProcessor colorLocatorRed;
    private ColorBlobLocatorProcessor colorLocatorYellow;

    private VisionPortal portal;
    /// END OF SAMPLE DETECTION VALUES

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
    private int pathState = 0;
    private final Pose parkingPose=new Pose(-55,70 - 0.5,Math.toRadians(90)); //parking


    double intakeMotorPower=0;
    double outakeMotorPower=0;

    // BASKET
    private final Pose startPose = new Pose(-10, 70, Math.toRadians(180)); //start

    // place in basket

    /*
    heading: 4.0289046655823935
intakerotate: 40.0
path state: 0
x: -52.47229868971458
y: 83.09612604576769
slides pos on descent -223
     */

    private boolean skipBasket = false;
    private boolean readyToDetect = false;


    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    //private Path startPath,pickUpFirst,pickUpSecond,pickUpThird,pickUpFourth,pickUpFifth,scoreFirst,scoreSecond,scoreThird,scoreFourth,scoreFifth,parking;
    //private PathChain goToPickUpFirstSample,goToPickUpSecondSample,goToPickUpThirdSample;

    private PathChain strafePath;
    private Pose strafeTarget;
    private ColorBlobLocatorProcessor.Blob selectedBlob;
    private double rightEnd = 69;
    private double leftEnd = 71.5;

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

//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        startPath = new Path(new BezierLine(new Point(startPose), new Point(scoringBarPosePreloadSpecimen)));
//        startPath.setLinearHeadingInterpolation(startPose.getHeading(), scoringBarPosePreloadSpecimen.getHeading());

        return;

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    // from 1 - 100 is normal paths
    // from 100+ is scoring paths
    private final int slideExtensionTimer = 300;
    private final int basketDropTimer = 75;



    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        resetStuff();
        isRobotInAuto = true;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();


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

        colorLocatorBlue = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(FTC_BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(0)// Smooth the transitions between different colors in image
                .setErodeSize(2)
                .build();
        colorLocatorRed = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(FTC_RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(0)                               // Smooth the transitions between different colors in image
                .setErodeSize(2)
                .build();
        colorLocatorYellow = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(FTC_YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(0)                               // Smooth the transitions between different colors in image
                .setErodeSize(2)
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocatorRed)
                .addProcessor(colorLocatorBlue)
                .addProcessor(colorLocatorYellow)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .setShowStatsOverlay(false)
                .build();

        //selectedBlob = selectFromDetection();
        readyToDetect = true;
    }



    public void robotDoStuff(){
        //risky
        follower.update();


        /*
        //ifs
        if(needsToExtraExtend && outtakeIsInNeedToExtraExtendClawTimer + 400 < System.currentTimeMillis()){
            needsToExtraExtend = false;
            outtakeClawServoPos = outtakeClawServoExtraExtendedPos;
        }

        //intake stop spinning
        if(shouldStopIntakeCabinSpinningAfterTakig && shouldStopIntakeCabinSpinningAfterTakigTimer + 500 < System.currentTimeMillis()){
            intakeSpinMotorPow = -0.8;
            shouldStopIntakeCabinSpinningAfterTakig = false;
            hasSmolOutputed = true;
            hasSmolOutputedTimer = System.currentTimeMillis();
        }
        //and then stop the power stuff
        if(hasSmolOutputed && hasSmolOutputedTimer + 50 <System.currentTimeMillis()){
            intakeCabinTransferPosition();
            if(isInSpecimenState){
                intakeCabinFullInBot();
            }
            hasSmolOutputed = false;
        }

        */

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

        robotTelemetry(640-selectedBlob.getBoxFit().center.x);
    }


    public void waitWhile(int timeToWait) {
        long iniTime = System.currentTimeMillis();
        while(iniTime + timeToWait > System.currentTimeMillis()){
            robotDoStuff();
        }
    }


    void robotTelemetry(double targetStrafe){
        tel.addData("path state", pathState);
        tel.addData("x", follower.getPose().getX());
        tel.addData("y", follower.getPose().getY());
        tel.addData("heading", follower.getPose().getHeading());
        tel.addData("intakePivotServoPos",intakePivotServoPos);
        tel.addData("outtakeTargetPos",outtakeExtendMotorTargetPos);
        tel.addData("outtakeDirection",outakeMotorPower);
        tel.addData("outtakeCurrenPos",outakeLeftMotor.getCurrentPosition());
        tel.addData("sensor color",currentStateOfSampleInIntake);
        tel.addData("follower is busy",follower.isBusy());
        tel.addData("intake motor", intakeMotor.getCurrentPosition());
        tel.addData("target strafe", targetStrafe);
        tel.addData("strafe targett", strafeTarget);
        Drawing.drawDebug(follower);
        FtcDashboard.getInstance().startCameraStream(portal, 30);
        dashboardTelemetry.update();
        tel.update();
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        //ColorBlobLocatorProcessor.Blob selectedBlob = selectFromDetection();

        // These loop the movements of the robot
        follower.update();

        robotDoStuff();


        // Feedback to Driver Hub
        robotTelemetry(selectedBlob.getBoxFit().center.x);
        //follower.holdPoint(new Pose(-10,60 + (71.5-60)*selectedBlob.getBoxFit().center.x/640, Math.toRadians(180)));
    }

    public ColorBlobLocatorProcessor.Blob selectFromDetection(){
        ColorBlobLocatorProcessor.Blob selectedBlob = null;

        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();
        blobs.addAll(colorLocatorRed.getBlobs());
        blobs.addAll(colorLocatorBlue.getBlobs());
        blobs.addAll(colorLocatorYellow.getBlobs());
        AtomicInteger i = new AtomicInteger(1);
        colorLocatorRed.getBlobs().forEach( blob -> {
            if(blob.getBoxFit() != null) {
                //telemetry.addData("Red Blob " + i + " Size Y", blob.getBoxFit().size.height);
                //telemetry.addData("Red Blob " + i + " Size X", blob.getBoxFit().size.width);
                //telemetry.addData("Red Blob " + i + " Angle", blob.getBoxFit().angle);
                i.getAndIncrement();
            }
        });
        AtomicInteger j = new AtomicInteger(1);
        colorLocatorYellow.getBlobs().forEach( blob -> {
            if(blob.getBoxFit() != null) {

                //telemetry.addData();
                //telemetry.addData("Yellow Blob " + j + " Size X", blob.getBoxFit().size.width);
                //telemetry.addData("Yellow Blob " + j + " Angle", blob.getBoxFit().angle);
                j.getAndIncrement();
            }
        });
        AtomicInteger k = new AtomicInteger(1);
        colorLocatorBlue.getBlobs().forEach( blob -> {
            if(blob.getBoxFit() != null) {
                //telemetry.addData("Blue Blob " + k + " Size Y", blob.getBoxFit().size.height);
                //telemetry.addData("Blue Blob " + k + " Size X", blob.getBoxFit().size.width);
                //telemetry.addData("Blue Blob " + k + " Angle", blob.getBoxFit().angle);
                k.getAndIncrement();
            }
        });


        // Variable to hold the blob with the maximum y coordinate
        int minDistance = Integer.MAX_VALUE; // Initialize maxY to the smallest possible integer

        int xCenter = 640;
        int yCenter = 0;

        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            String color = "Cat";
            if (colorLocatorRed.getBlobs().contains(b))
                color = "Red";
            else if (colorLocatorBlue.getBlobs().contains(b))
                color = "Blue";
            else if (colorLocatorYellow.getBlobs().contains(b))
                color = "Yellow";
            else
                continue;

            RotatedRect boxFit = b.getBoxFit();
            telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y) + " " + b.getBoxFit().angle + " " + color);

            dashboardTelemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y) + " " + b.getBoxFit().angle + " " + color);

            int xDistance = (int) boxFit.center.x - xCenter;
            int yDistance = (int) boxFit.center.y - yCenter;
            int distance = xDistance*xDistance + yDistance*yDistance;

            // Check if the current blob has a higher y coordinate than the current maxY
            if (distance < minDistance) {
                minDistance = distance; // Update maxY
                selectedBlob = b; // Update selectedBlob to the current blob
            }
        }

        return selectedBlob;
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        if(readyToDetect){
            selectedBlob = selectFromDetection();
            if (selectedBlob!=null){
                robotTelemetry(selectedBlob.getBoxFit().center.x);
            }
        }
        selectedBlob = selectFromDetection();
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
        selectedBlob = selectFromDetection();
        strafeTarget = new Pose(-10,rightEnd + (leftEnd-rightEnd)*selectedBlob.getBoxFit().center.x/640, Math.toRadians(180));
        strafePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(strafeTarget)))
                .setLinearHeadingInterpolation(startPose.getHeading(), strafeTarget.getHeading())
                .build();
        follower.holdPoint(strafeTarget);

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }




}
