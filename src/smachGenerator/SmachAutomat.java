package smachGenerator;

import java.io.File;
import java.io.PrintWriter;
import java.rmi.AlreadyBoundException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;

import javax.naming.directory.NoSuchAttributeException;

/**
 * Creates the python Source-Code for a Smach state machine for ROS (Robot
 * Operation System) It can be stored to a file
 * 
 * @author Marvin Lindenberg
 * 
 */
public class SmachAutomat {

	private ArrayList<? extends ISmachableState> states = new ArrayList<>();
	private LinkedList<String> smachStates = new LinkedList<>();
	private int initialStateIndex;
	private SmachableSensors sensors;
	private SmachableActuators actuators;
	private String pkg;

	/**
	 * Constructs a Smach state machine form the given states
	 * 
	 * @param states
	 *            used to create the state machine
	 * @param sensors
	 *            containing all {@link ISmachableSensor}s that are used in the
	 *            {@link ISmachableTransition}s
	 * @param actuators
	 *            containing all {@link ISmachableActuator}s that are used in
	 *            the {@link ISmachableState}s
	 * @param pkg
	 *            the package the state machine is stored in, to load the right
	 *            manifest
	 * 
	 * @throws NoSuchAttributeException
	 *             if sensors are used that are not specified in the
	 *             {@link ISmachableSensors} object.
	 */
	public SmachAutomat(ArrayList<? extends ISmachableState> states,
			SmachableSensors sensors, SmachableActuators actuators, String pkg)
			throws NoSuchAttributeException {
		this.states = states;
		this.sensors = sensors;
		this.actuators = actuators;
		this.pkg = pkg;
		for (ISmachableState s : states) {
			if (s.isInitialState()) {
				initialStateIndex = states.indexOf(s);
			}
			smachStates.add(getSmachState(s));
		}
	}

	private String getImports() {
		String imports = "#!/usr/bin/env python\n\n";
		imports += "import roslib; roslib.load_manifest('" + pkg + "')\n";
		imports += "import rospy\n";
		imports += "import smach\n";
		imports += "import smach_ros\n\n";

		// messege dependencies
		HashSet<String> deps = sensors.getMsgDeps();
		deps.addAll(actuators.getMsgDeps());
		for (String dep : deps) {
			imports += dep + "\n";
		}
		return imports;
	}

	/**
	 * Rewrites the ISmachableState s to a Smach state
	 * 
	 * @param s
	 *            ISmachableState that will be converted to a smach state
	 * @throws NoSuchAttributeException
	 */
	private String getSmachState(ISmachableState s)
			throws NoSuchAttributeException {
		String state = "class " + s.getText().replace(" ", "")
				+ "(smach.State):\n";
		// create __init__ function for state
		state += "\tdef __init__(self):\n";
		state += "\t\tsmach.State.__init__(self, outcomes=[";
		// get transitions for __init__ function
		if (s.getTransitions().size() > 0) {
			for (ISmachableTransition t : s.getTransitions()) {
				state += "'" + t.getLabel() + "',";
			}
			state = state.substring(0, state.length() - 1) + "])\n\n";
		} else {
			state += "])\n\n";
		}

		// create execute function for the state
		state += "\tdef execute(self, userdata):\n";
		state += "\t\trospy.loginfo('Executing state " + s.getText() + "')\n";
		// add global declaration for all sensor variables needed
		for (ISmachableTransition t : s.getTransitions()) {
			ISmachableGuard g = t.getSmachableGuard();
			for (String sensor : g.getSensorNames()) {
				if (sensor.startsWith("DIFFERENCE_")) {
					String sensorNames[] = sensor.replace("DIFFERENCE_", "")
							.split("_");
					if (sensors.getSensor(sensorNames[0]) != null
							&& sensors.getSensor(sensorNames[1]) != null) {
						state += "\t\tglobal " + sensorNames[0]
								+ "\n\t\tglobal " + sensorNames[1] + "\n";
					}
				}
				state += "\t\tglobal " + sensor + "\n";
			}
		}
		
		//add actions
		for (ISmachableAction a : s.getActions()) {
			ISmachableActuator actuator = actuators.getActuator(a.getKey());
			for (String str : actuator.getPublishMessage(a)){
				state += "\t\t" + str + "\n";
			}
		}
			
		
		// check for transition
		state += "\n\t\twhile not rospy.is_shutdown():\n";
		for (ISmachableTransition t : s.getTransitions()) {
			ISmachableGuard guard = t.getSmachableGuard();
			if (guard.getSensorNames().size() > 0) {
				state += "\t\t\tif(";
				for (int i = 0; i < guard.getSensorNames().size(); i++) {
					if (guard.getSensorNames().get(i).startsWith("DIFFERENCE_")) {
						String sensorNames[] = guard.getSensorNames().get(i)
								.replace("DIFFERENCE_", "").split("_");
						if (sensors.getSensor(sensorNames[0]) != null
								&& sensors.getSensor(sensorNames[1]) != null) {
							state += sensorNames[0] + "-" + sensorNames[1]
									+ guard.getOperators().get(i)
									+ guard.getCompValues().get(i) + " and ";
						}
					} else if (sensors.getSensor(guard.getSensorNames().get(i)) != null) {
						state += sensors.getSensor(
								guard.getSensorNames().get(i))
								.getTransitionCondition(
										guard.getOperators().get(i),
										guard.getCompValues().get(i))
								+ " and ";
					}
				}
				state = state.substring(0, state.length() - 5) + "):\n\t";
			}
			state += "\t\t\treturn '" + t.getLabel() + "'\n";
		}
		state += "\t\t\trospy.sleep(0.01)\n";
		return state;
	}

	private String getSmachStateMachine(String sm) {
		if (states.size() > 0) {
			// prepare all states to add them to the Smach state machine
			ArrayList<String> stateToAdd = new ArrayList<>();
			for (int i = 0; i < states.size(); i++) {
				String state = "smach.StateMachine.add('"
						+ states.get(i).getText().replace(" ", "") + "', "
						+ states.get(i).getText().replace(" ", "")
						+ "(), transitions={";
				for (ISmachableTransition t : states.get(i).getTransitions()) {
					state += "'" + t.getLabel() + "':'"
							+ t.getFollowerState().getText().replace(" ", "")
							+ "',";
				}
				if (states.get(i).getTransitions().size() > 0) {
					state = state.substring(0, state.length() - 1) + "})";
				} else {
					state += "})";
				}
				stateToAdd.add(state);
			}
			// create state machine and add states in correct order
			String stateMachine = "\t" + sm
					+ " = smach.StateMachine(outcomes=[])\n";
			stateMachine += "\twith " + sm + ":\n";
			stateMachine += "\t\t" + stateToAdd.get(initialStateIndex) + "\n";
			for (int i = 0; i < stateToAdd.size(); i++) {
				if (i != initialStateIndex) {
					stateMachine += "\t\t" + stateToAdd.get(i) + "\n";
				}
			}
			return stateMachine;
		}
		return "";
	}

	private String getMainMethode() {
		String main = "if __name__ == '__main__':\n";
		main += "\trospy.init_node('zusmoro_state_machine')\n";
		for (String subSetup : sensors.getSubscriberSetups()) {
			main += "\t" + subSetup;
		}
		main += getSmachStateMachine("sm");
		// including possibility to use Smach_viewer
		main += "\tsis = smach_ros.IntrospectionServer('Beep_State_Server', sm, '/SM_ROOT')\n";
		main += "\tsis.start()\n\tsm.execute()\n";
		main += "\trospy.spin()\n\tsis.stop()";
		return main;
	}

	/**
	 * Stores the smach automat as an executable python-program to the given
	 * filename
	 * 
	 * @param fileName
	 *            the python-programm will be stored in.
	 * @return true, if constructing and saving was successfully. False if there
	 *         were problems with Topics, names, or the saving progress.
	 * @throws NoSuchAttributeException
	 * @throws AlreadyBoundException
	 */
	public boolean saveToFile(File file) {
		try {
			String pythonNode = getImports() + "\n\n";
			// define global sensor variables
			for (ISmachableSensor sensor : sensors) {
				pythonNode += sensor.getName() + " = 0\n";
			}
			// define global actuator publisher
			for (String pub : actuators.getPublisherSetups()) {
				pythonNode += pub + "\n";
			}

			pythonNode += "\n\n";

			for (int i = 0; i < smachStates.size(); i++) {
				pythonNode += smachStates.get(i) + "\n";
			}
			for (String cb : sensors.getCallbacks()) {
				pythonNode += cb + "\n";
			}
			pythonNode += getMainMethode();
			PrintWriter out = new PrintWriter(file);
			out.print(pythonNode);
			out.close();
			file.setExecutable(true);
		} catch (Exception e) {
			e.printStackTrace();
			return false;
		}
		return true;
	}

}
