package pedroPathing.States;

import static pedroPathing.PositionStorage.IntakeServoColectPos;
import static pedroPathing.PositionStorage.gravityAdder;
import static pedroPathing.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.PositionStorage.isIntakeStateExtended;
import static pedroPathing.PositionStorage.pickUpAngleRo2V2Adder;
import static pedroPathing.PositionStorage.stateStringIntake;
import static pedroPathing.PositionStorage.wasActivePastActiveIntake;
import static pedroPathing.PositionStorage.wasActiveintake;
import static pedroPathing.PositionStorage.wasBambuExtended;
import static pedroPathing.PositionStorage.wasIntakeStateExtended;

public class IntakeStateExtendedHM implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateExtended...");
        // Set servo and motor positions for this state
        stateStringIntake = "IntakeStateExtended HUMAN PLAYER SAMPLE OUT";
        intakeRotateServoPosition = IntakeServoColectPos + pickUpAngleRo2V2Adder;
        if(gravityAdder==0)
            gravityAdder = 7;
    }
}
