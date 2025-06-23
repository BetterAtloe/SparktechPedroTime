package pedroPathing;

@com.acmerobotics.dashboard.config.Config
public class OrganizedPositionStorage {
    // MISC
    public static double PIDincrement;
    public static double gravityAdder;

    // chassis / sasiu
    public static double chassisBackLeftPow = 0;
    public static double chassisBackRightPow = 0;
    public static double chassisFrontLeftPow = 0;
    public static double chassisFrontRightPow = 0;

    // intake
    public static double intakePivotServoPos;
    public static double intakeSpinMotorPow;
    public static double intakeExtendMotorTargetPos=0;
    public static double intakeTargetPosAdder;
    public static double intakeGravitySubtractor=0;

    // outtake
    public static double outtakePivotServoPos;
    public static double outtakeClawServoPos;
    public static double outtakeExtendMotorTargetPos=0;

    public static double outtakeTargetPosAdder;






    // CONSTANTE*


    //Outtake

    //outtake claw
    public static double outtakeClawServoExtendedPos = 128;
    public static double outtakeClawServoExtraExtendedPos = 208;
    public static double outtakeClawServoRetractedPos = 15;


    // outtake pivot
    public static double outtakePivotServoWallPickupPos = 297;
    public static double outtakePivotServoHighRungHangPos = 178;
    public static double outtakePivotServoBasketPos = 52;
    public static double outtakePivotServoTransferPos = 210;
    public static double outtakePivotServoStandByPos = outtakePivotServoHighRungHangPos ;


    // outtake sliders

    public static double outtakeMotorMaxPos = 2100;

    public static double outtakeMotorMaxPosLowerBasket = 800;
    public static double outtakeSliderSpecimenHangPos = 1100;
    public static double autoOuttakeSliderSpecimenHangPos = 950;
    public static double outtakeSlidersWallPickPos = 690;
    public static double outtakeMotorActualZeroPos = 0;
    public static double outtakeMotorStandByPos = 1000;
    public static double tempOuttakeAPosition = 0;




    //intake

    //intake pivot
    public static double intakePivotServoPickupPos = 181;   // old 206
    public static double intakePivotServoOutputTruBotPos = 0;
    public static double intakePivotServoTransferPos = 130;
    public static double tempIntakeTargetPastPosDifrence = 135;
    public static double tempIntakeAPosition = 0;



    // BOOLEANS

    //misc
    public static boolean isYellowSampleNotGood = false;
    public static boolean reverseGamepad2 = false;
    public static boolean isRobotInAuto = false;
    public static boolean isInLowerBasketState = false;
    public static boolean isColorSensorNotInUse = false;
    public static boolean somethingFailed = false;


    //is pressed
    public static boolean isPressedA1 = false;
    public static boolean isPressedA2 = false;
    public static boolean isPressedB1 = false;
    public static boolean isPressedB2 = false;
    public static boolean isPressedX1 = false;
    public static boolean isPressedX1v2 = false;
    public static boolean isPressedX2 = false;
    public static boolean isPressedY1 = false;
    public static boolean isPressedY2 = false;
    public static boolean isPressedDL1 = false;
    public static boolean isPressedD2Up = false;


    //intake stuff
    public static boolean isAfterIntakeBeenDownColecting = false;
    public static boolean isIntakeOutputting = false;
    public static boolean isAfterBotHasBeenOutputting = false;
    public static boolean isAfterOuttakeClawClosedAfterTransfer = false;
    public static boolean isIntakeOutputtingManual = false;
    public static boolean isIntakeSpinMOtorAfterJustTaking = false;
    public static boolean isInCaseOfNotIntakeInBot = false;
    public static boolean isInSpecimenState = false;
    public static boolean isTimeToRefreshOutptingTime = false;
    public static boolean isInPositionToRaiseOuttakeInOrderToEvadeIntake = false;
    public static boolean shouldStopIntakeCabinSpinningAfterTakig = false;
    public static boolean isAfterTakingTakeySpiny = false;
    public static boolean wasIntakeCabinTruBotOutputting = false;
    public static boolean isIntakeInPositionToOutputTruBot = false;
    public static boolean hasIntakeOutputedTruBot = false;
    public static boolean hasSmolOutputed = false;
    public static boolean didTransfera = false;
    public static int basketStandbyState = 0;

    //outtake stuff
    public static boolean isAfterOuttakeScoredSpecimen = false;
    public static boolean isAfterOuttakeScoredBasketSample = false;
    public static boolean isAfterOuttakeClosedClawAtWallSpecimen = false;
    public static boolean isAtStateOfLettingBasketSampleGo = false;
    public static boolean isInNeedToGoToSpecimenTransferPos = false;
    public static boolean isOuttakeInPositionToGoDown = false;
    public static boolean outtakeIsInNeedToExtraExtendClaw = false;
    public static boolean isOuttakeInPositionToCloseClawForTransfer = false;
    public static boolean isOuttakeInPositionToGoToStandBy = false;
    public static boolean needsToExtraExtend = false;
    public static boolean shouldAutoCollect = false;
    public static boolean shouldTransfer = false;
    public static boolean shouldSpecimenTransfer = false;
    public static boolean isOuttakeAfterOutputedTruBot = false;
    public static boolean justTransfered = false;





    //LONGS / TIMERS

    //auto stuff
    public static long autoTimer;


    //intake stuff

    public static long intakeOutputtingTimer;
    public static long intakeAfterTransferClosedClawTimer;
    public static long intakeOutputtingTimerManual;
    public static long intakeSpinMotorMorePowerAfterTakingTimer;
    public static long isIntakeInBotTimer;

    public static long timeSinceStartedMovingForTruBotOutput;
    public static long waitingForOuttakeToEvadeIntakeTimer;
    public static long shouldStopIntakeCabinSpinningAfterTakigTimer;
    public static long hasSmolOutputedTimer;
    public static long hasPressedXTimer;
    public static long didTransferTime;



    //outtake stuff
    public static long outtakeSpecimenAfterScoreTimer;
    public static long outtakeAfterBasketSampleScoreTimer;
    public static long outtakeAfterHasClosedClawAtWallSpecimenTimer;
    public static long beforeOuttakeGoDownTimer;
    public static long outtakeIsInNeedToExtraExtendClawTimer;
    public static long outtakeCloseClawInTransferTimer;
    public static long outtakeGoToStandByTimer;
    public static long isOuttakeAfterOutputedTruBotTimer;



    public static void resetStuff() {
        //toggles
        Toggle.toggled = false;
        Toggle.toggle_var = false;
        Toggle.toggledButton2 = false;
        Toggle.toggledVarButton2 = false;
        isRobotInAuto = false;
        isInLowerBasketState = false;
        somethingFailed = false;

        // MISC
        PIDincrement = 0;
        gravityAdder = 0;

        // intake
        intakePivotServoPos = intakePivotServoTransferPos;
        intakeExtendMotorTargetPos = 0;
        intakeTargetPosAdder = 0;
        intakeGravitySubtractor = 0;
        tempIntakeAPosition = 0;

        // outtake
        outtakePivotServoPos = outtakePivotServoTransferPos;
        outtakeClawServoPos = outtakeClawServoExtendedPos;
        outtakeExtendMotorTargetPos = 0;
        outtakeTargetPosAdder = 0;
        tempOuttakeAPosition = 0;

        // BOOLEANS
        isYellowSampleNotGood = false;
        reverseGamepad2 = false;

        // is pressed
        isPressedA1 = false;
        isPressedA2 = false;
        isPressedB1 = false;
        isPressedB2 = false;
        isPressedX1 = false;
        isPressedX2 = false;
        isPressedY1 = false;
        isPressedY2 = false;
        isPressedDL1 = false;
        isPressedD2Up = false;

        // intake stuff
        isAfterIntakeBeenDownColecting = false;
        isIntakeOutputting = false;
        isAfterBotHasBeenOutputting = false;
        isAfterOuttakeClawClosedAfterTransfer = false;
        isIntakeOutputtingManual = false;
        isInSpecimenState = false;
        shouldStopIntakeCabinSpinningAfterTakig = false;
        isAfterTakingTakeySpiny = false;
        wasIntakeCabinTruBotOutputting = false;
        isIntakeInPositionToOutputTruBot = false;
        hasIntakeOutputedTruBot = false;
        didTransfera = false;
        hasSmolOutputed = false;

        // outtake stuff
        isAfterOuttakeScoredSpecimen = false;
        isAfterOuttakeScoredBasketSample = false;
        isAfterOuttakeClosedClawAtWallSpecimen = false;
        isAtStateOfLettingBasketSampleGo = false;
        outtakeIsInNeedToExtraExtendClaw = false;
        isOuttakeInPositionToCloseClawForTransfer = false;
        isOuttakeInPositionToGoToStandBy = false;
        isOuttakeAfterOutputedTruBot = false;
        justTransfered = false;

        // LONGS / TIMERS
        outtakeSpecimenAfterScoreTimer = 0;
        outtakeAfterBasketSampleScoreTimer = 0;
        outtakeAfterHasClosedClawAtWallSpecimenTimer = 0;
        intakeOutputtingTimer = 0;
        intakeAfterTransferClosedClawTimer = 0;
    }
}