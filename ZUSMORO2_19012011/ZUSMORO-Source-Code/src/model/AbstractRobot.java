package model;

import java.io.IOException;
import java.util.ArrayList;

import view.AbstractStatePanel;
import view.AbstractTransitionPanel;

public abstract class AbstractRobot {
	public AbstractRobot() {
		//nothing
	}
	public abstract boolean connect(String connectTo);
	public abstract void disconnect();
	public abstract int getVariableValue(String variable);
	public abstract void executeActions(ArrayList<Action> actions) throws IOException;
	public abstract void updateSensors() throws IOException;
	public abstract void stop();
	public abstract int getDesiredAdditionalTimeout();
	public abstract AbstractStatePanel getStatePanel(State state);
	public abstract AbstractTransitionPanel getTransitionPanel(Transition trans);
}
