package pedroPathing.States;

import static pedroPathing.States.PositionStorage.*;

import pedroPathing.Toggle;

public class IntakeStateRetracted implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateRetracted...");
        // Set servo and motor positions for this state
        //telemetry.addData("IntakeStateRectracted",true);
        gravityAdder = 0;
        stateStringIntake = "IntakeStateRetracted";
        isIntakeStateRectracted = true;
        intakeRotateServoPosition = 30;
        intakeTargetPos = intakeTransferAngles;
        intakeServoPower = 0;
        if(wasActivePastActiveIntake){
            wasActivePastActiveIntake = false;
            Toggle.toggledUp = false;
            Toggle.toggle_varUp = false;
            Toggle.toggledLeft = false;
            Toggle.toggle_varLeft = false;
            Toggle.toggledDown = false;
            Toggle.toggle_varDown = false;
            Toggle.toggledRight = false;
            Toggle.toggle_varRight = false;
        }
    }
}
