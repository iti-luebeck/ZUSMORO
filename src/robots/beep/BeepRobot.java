package robots.beep;

import java.io.IOException;
import java.util.ArrayList;

import view.AbstractStatePanel;
import view.AbstractTransitionPanel;
import model.AbstractRobot;
import model.Action;
import model.State;
import model.Transition;

public class BeepRobot extends AbstractRobot {

	@Override
	public boolean connect(String connectTo) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void disconnect() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int getVariableValue(String variable) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void executeActions(ArrayList<Action> actions) throws IOException {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void updateSensors() throws IOException {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int getDesiredAdditionalTimeout() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public AbstractStatePanel getStatePanel(State state) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public AbstractTransitionPanel getTransitionPanel(Transition trans) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int getUnAcknowledgedCmds() {
		// TODO Auto-generated method stub
		return 0;
	}

}
