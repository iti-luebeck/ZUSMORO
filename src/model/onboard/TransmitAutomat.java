package model.onboard;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import view.MainFrame;

import model.Action;
import model.ActionController;
import model.Automat;
import model.State;
import model.Transition;
import model.bool.BooleanExpression;
import model.bool.HugeAnd;
import model.bool.Variable;
import model.bool.Variable.Operator;

public class TransmitAutomat implements TransmissionJob {

	protected int numberOfGuards;
	protected int numberOfStates;
	protected int numberOfTransitions;
	protected int initialState;

	int pointer;
	protected List<String> sequence;
	protected List<State> states;
	protected List<Transition> trans;
	protected List<Variable> guards;

	public TransmitAutomat(Automat automat) {
		Automat a = transform(automat);
		states = a.getStates();
		sequence = new LinkedList<String>();
		trans = new LinkedList<Transition>();
		guards = new LinkedList<Variable>();
		for (State s : states) {
			for (Transition t : s.getTransitions()) {
				trans.add(t);
				for (BooleanExpression b : ((HugeAnd) t.getGuard())
						.getOperands()) {
					guards.add((Variable) b);
				}
			}
		}
		numberOfStates = states.size();
		numberOfTransitions = trans.size();
		numberOfGuards = guards.size();
		//numberOfGuards = 0;

		initialState = states.indexOf(a.getInitialState());
		sequence.addAll(stateListStrings(false));
		sequence.addAll(transitionListStrings(false));
		// numberOfGuards = maxGuardIndex +1; implizit durch
		// transitionListStrings
		sequence.addAll(guardListStrings(false));
		sequence.add("A" + initialState);
		MainFrame.automat=automat;
		pointer = 0;
	}

	private State mapState(Automat a1, Automat a2, State s1) {
		return a2.getStates().get(a1.getStates().indexOf(s1));
	}

	protected Automat transform(Automat automat) {
		Automat a = new Automat();
		int x = 0;
		for (State s2 : automat.getStates()) {
			State s = new State("" + x++, s2.isInitialState());
			s.setActions(s2.getActions());
			a.addState(s);
		}
		for (State s2 : automat.getStates()) {
			State s=mapState(automat, a, s2);
			ArrayList<Transition> extraTransitions = new ArrayList<Transition>();
			for (Transition t3 : s2.getTransitions()) {
				Transition t = new Transition(mapState(automat, a, t3
						.getRootState()), mapState(automat, a, t3
						.getFollowerState()));
				t.setGuard(t3.getGuard());
				HugeAnd operands = new HugeAnd();
				if (t.getGuard() instanceof HugeAnd) {
					boolean useTimer = false;
					Variable timerVar = new Variable("TIMER", Operator.EQUAL, 0);
					for (BooleanExpression b : ((HugeAnd) t.getGuard())
							.getOperands()) {
						Variable v = (Variable) b;
						if (v.getVariableName().equalsIgnoreCase("timer")) {
							timerVar = v;
							useTimer = true;
						} else {
							operands.addOperand(v);
						}

						if (useTimer) {
							Operator op = timerVar.getOperator();
							int compValue = timerVar.getCompValue();
							int timerms = compValue % 30000;
							int timer30s = compValue / 30000;

							switch (op) {
							case BIGGER: {
								Transition t2 = new Transition(
										t.getRootState(), t.getFollowerState());
								HugeAnd op2 = new HugeAnd();
								for (BooleanExpression b2 : operands
										.getOperands()) {
									op2.addOperand(b2);
								}
								op2.addOperand(new Variable("TIMER30S",
										Operator.EQUAL, timer30s));
								op2.addOperand(new Variable("TIMERMS",
										Operator.BIGGER, timerms));
								t2.setGuard(op2);
								extraTransitions.add(t2);
								operands.addOperand(new Variable("TIMER30S",
										Operator.BIGGER, timer30s));
								break;
							}
							case BIGGER_EQUAL: {
								Transition t2 = new Transition(
										t.getRootState(), t.getFollowerState());
								HugeAnd op2 = new HugeAnd();
								for (BooleanExpression b2 : operands
										.getOperands()) {
									op2.addOperand(b2);
								}
								op2.addOperand(new Variable("TIMER30S",
										Operator.EQUAL, timer30s));
								op2.addOperand(new Variable("TIMERMS",
										Operator.BIGGER_EQUAL, timerms));
								t2.setGuard(op2);
								extraTransitions.add(t2);

								operands.addOperand(new Variable("TIMER30S",
										Operator.BIGGER, timer30s));
								break;
							}
							case EQUAL: {
								operands.addOperand(new Variable("TIMERMS",
										Operator.EQUAL, timerms));
								operands.addOperand(new Variable("TIMER30S",
										Operator.EQUAL, timer30s));
								break;
							}
							case NOT_EQUAL: {
								Transition t2 = new Transition(
										t.getRootState(), t.getFollowerState());
								HugeAnd op2 = new HugeAnd();
								for (BooleanExpression b2 : operands
										.getOperands()) {
									op2.addOperand(b2);
								}
								op2.addOperand(new Variable("TIMERMS",
										Operator.NOT_EQUAL, timerms));
								t2.setGuard(op2);
								extraTransitions.add(t2);

								operands.addOperand(new Variable("TIMER30S",
										Operator.NOT_EQUAL, timer30s));
								break;
							}
							case SMALLER_EQUAL: {
								Transition t2 = new Transition(
										t.getRootState(), t.getFollowerState());
								HugeAnd op2 = new HugeAnd();
								for (BooleanExpression b2 : operands
										.getOperands()) {
									op2.addOperand(b2);
								}
								op2.addOperand(new Variable("TIMER30S",
										Operator.EQUAL, timer30s));
								op2.addOperand(new Variable("TIMERMS",
										Operator.SMALLER_EQUAL, timerms));
								t2.setGuard(op2);
								extraTransitions.add(t2);

								operands.addOperand(new Variable("TIMER30S",
										Operator.SMALLER, timer30s));
								break;
							}
							case SMALLER: {
								Transition t2 = new Transition(
										t.getRootState(), t.getFollowerState());
								HugeAnd op2 = new HugeAnd();
								for (BooleanExpression b2 : operands
										.getOperands()) {
									op2.addOperand(b2);
								}
								op2.addOperand(new Variable("TIMER30S",
										Operator.EQUAL, timer30s));
								op2.addOperand(new Variable("TIMERMS",
										Operator.SMALLER, timerms));
								t2.setGuard(op2);
								extraTransitions.add(t2);

								operands.addOperand(new Variable("TIMER30S",
										Operator.SMALLER, timer30s));
								break;
							}

							default:
								break;
							}
						}
					}

				}

				t.setGuard(operands);
				s.addTransition(t);
			}
			for (Transition t : extraTransitions) {
				s.addTransition(t);
			}
			//a.addState(s);

		}
		a.setInitialState(a.getStates().get(
				automat.getStates().indexOf(automat.getInitialState())));
		return a;
	}

	public void printTest() {
		for (String s : sequence) {
			System.out.println(s);
		}
	}

	@Override
	public double completionStatus() {
		return pointer / (double) sequence.size();
	}

	@Override
	public void removeFirst() {
		pointer++;
	}

	@Override
	public String getFirst() {
		return sequence.get(pointer);
	}

	@Override
	public boolean isComplete() {
		return (pointer >= sequence.size());
	}

	@Override
	public boolean check(String lastMsgRec, String lastMsgSent) {
		System.out.println(lastMsgRec + "!=" + lastMsgSent);
		String compareString = null;

		char firstChar = lastMsgSent.charAt(0);
		int n = Integer.parseInt(lastMsgSent.substring(1));
		int restartAt = -1;
		// TODO geht das besser als switch case?

		switch (firstChar) {
		case 'y': {
			compareString = concatStringList(stateStrings(n, true));
			restartAt = sequence.indexOf("z" + n + ","
					+ states.get(n).getTransitions().size());
			break;
		}
		case 'Y': {
			if (n == numberOfStates) {
				compareString = concatStringList(stateListStrings(true));
			}
			restartAt = sequence.indexOf("Z" + numberOfStates);
			break;
		}
		case 'u': {
			compareString = concatStringList(transitionStrings(n, true));
			restartAt = sequence.indexOf(concatStringList(
					transitionStrings(n, true)).replaceAll("g.*", ""));
			break;
		}
		case 'U': {
			if (n == numberOfTransitions) {
				compareString = concatStringList(transitionListStrings(true));
			}
			restartAt = sequence.indexOf("T" + numberOfTransitions);
			break;
		}
		case 'C': {
			if (n == numberOfGuards) {
				compareString = concatStringList(guardListStrings(true));
			}
			restartAt = sequence.indexOf("B" + numberOfGuards);
			break;
		}
		case 'a': {
			if (n == initialState) {
				compareString = "A" + initialState;
			}
			restartAt = sequence.indexOf("A" + initialState);
			break;
		}
		default:
			break;
		}
		// TODO ende switch case
		if (compareString == null || !lastMsgRec.equals(compareString)) {
			System.out.println("Error");
			pointer = restartAt;
		}

		System.out.println("Correct:" + compareString);
		return lastMsgRec.equals(compareString);
	}

	protected String concatStringList(List<String> sl) {
		String ret = "";
		for (String s : sl) {
			ret += s;
		}
		return ret;
	}

	protected int searchActions(List<Action> actions, String string) {
		int value = 0;
		for (Action a : actions) {
			if (a.getKey().equals(string)) {
				value = a.getValue();
				break;
			}
		}
		return value;

	}

	protected int sensorToInt(String name) {
		final int IR = 0;
		final int UIR = 8;
		final int TIMERMS = 11;
		final int TIMER30S = 12;

		int res = 99;
		if (name.startsWith("IR")) {
			res = IR + Character.getNumericValue(name.charAt(2));
		} else if (name.startsWith("UIR")) {
			res = UIR + Character.getNumericValue(name.charAt(3));
		} else if (name.equalsIgnoreCase("timerms")) {
			res = TIMERMS;
		} else if (name.equalsIgnoreCase("timer30s")) {
			res = TIMER30S;
		} else if (name.startsWith("DIFFERENCE")) {
			String[] split = name.split("_");
			// getVar(c)
			// if c<0
			// then getVar(c):= getVar((-c)%100)-getVar((-c)/100)
			res = -100 * sensorToInt(split[2]) - sensorToInt(split[1]);
		}
		return res;
	}

	protected List<String> stateListStrings(boolean check) {
		List<String> stateListStrings = new LinkedList<String>();
		stateListStrings.add("Z" + numberOfStates);
		for (int i = 0; i < states.size(); i++) {
			stateListStrings.addAll(stateStrings(i, check));
		}
		if (!check) {
			stateListStrings.add("Y" + numberOfStates);
		}
		return stateListStrings;
	}

	protected List<String> stateStrings(int i, boolean check) {
		State state = states.get(i);
		LinkedList<String> stateStrings = new LinkedList<String>();
		ArrayList<Action> actions = state.getActions();
		ArrayList<Transition> transitions = state.getTransitions();
		stateStrings.add("z" + i + "," + transitions.size());
		/*
		 * alte Version ohne Regler int motorR = searchActions(actions,
		 * "MOTOR2"); if (check || motorR != 0) { stateStrings.add("r" +
		 * motorR); }
		 * 
		 * int motorL = searchActions(actions, "MOTOR1"); if (check || motorL !=
		 * 0) { stateStrings.add("l" + motorL); }
		 */
		ArrayList<Integer> l = new ArrayList<Integer>(2);
		l.add(2);
		l.add(1);
		for (int j : l) {
			int offset = searchActions(actions, "MOTOR" + j);
			int c = searchActions(actions, "MOTOR" + j + "CONTROLLER");
			if (check || offset != 0 || c != 0) {
				if (c == 0) {
					stateStrings.add((j == 2 ? "r" : "l") + offset
							+ ",0,0,0,0,0");
				} else {
					for (Action a : actions) {
						if (a.getKey().equalsIgnoreCase(
								"motor" + j + "controller")) {
							stateStrings
									.add(controllerString((ActionController) a));
						}
					}
				}

			}
		}

		LEDSet leds = new LEDSet();
		leds.set(actions);
		int ledValue = leds.getValue();
		if (check || ledValue != 0) {
			stateStrings.add("L" + ledValue);
		}

		for (Transition t : transitions) {
			stateStrings.add("G" + trans.indexOf(t));
		}

		if (!check) {
			stateStrings.add("y" + i);
		}

		return stateStrings;
	}

	private String controllerString(ActionController a) {
		String res;
		if (a.getKey().equalsIgnoreCase("motor1controller")) {
			res = "l";
		} else if (a.getKey().equalsIgnoreCase("motor2controller")) {
			res = "r";
		} else {
			return "Controller Unknown";
		}
		res += a.getOffset() + "," + a.getMin() + "," + a.getMax() + ","
				+ a.getKpr() + "," + sensorToInt(a.getVar()) + ","
				+ a.getCompvalue();
		return res;
	}

	protected List<String> transitionListStrings(boolean check) {
		LinkedList<String> transitionListStrings = new LinkedList<String>();
		transitionListStrings.add("T" + numberOfTransitions);
		for (int i = 0; i < trans.size(); i++) {
			transitionListStrings.addAll(transitionStrings(i, check));
		}
		if (!check) {
			transitionListStrings.add("U" + numberOfTransitions);
		}
		return transitionListStrings;
	}

	protected List<String> transitionStrings(int i, boolean check) {
		Transition transition = trans.get(i);
		List<String> transitionStrings = new LinkedList<String>();
		BooleanExpression[] operands = ((HugeAnd) transition.getGuard())
				.getOperands();
		transitionStrings.add("t" + i + "," + operands.length + ","
				+ states.indexOf(transition.getFollowerState()));
		for (BooleanExpression b : operands) {
			int guardIndex = guards.indexOf(b);
			if (guardIndex + 1 > numberOfGuards) {
				numberOfGuards = guardIndex + 1;
			}
			transitionStrings.add("g" + guardIndex);
		}
		if (!check) {
			transitionStrings.add("u" + i);
		}
		return transitionStrings;
	}

	protected List<String> guardListStrings(boolean check) {
		List<String> guardListStrings = new LinkedList<String>();
		guardListStrings.add("B" + numberOfGuards);
		for (int i = 0; i < numberOfGuards; i++) {
			Variable v = guards.get(i);
			guardListStrings
					.add("b" + i + "," + v.getOperator().getOperatorNumber()
							+ "," + sensorToInt(v.getVariableName()) + ","
							+ v.getCompValue());
		}
		if (!check) {
			guardListStrings.add("C" + numberOfGuards);
		}
		return guardListStrings;
	}

}
