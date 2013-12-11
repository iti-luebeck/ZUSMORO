package model;

import smachGenerator.ISmachableAction;


public class Action implements ISmachableAction {

	private String actuatorName;
	private int value;

	public Action(String key, int value) {
		this.actuatorName = key;
		this.value = value;
	}

	/* (non-Javadoc)
	 * @see model.ISmachableAction#getKey()
	 */
	@Override
	public String getActuatorName() {
		return actuatorName;
	}

	public void setActuatorName(String key) {
		this.actuatorName = key;
	}

	/* (non-Javadoc)
	 * @see model.ISmachableAction#getValue()
	 */
	@Override
	public int getValue() {
		return value;
	}
	public void setValue(int value) {
		this.value = value;
	}

	@Override
	public boolean equals(Object o) {
		if (o instanceof Action) {
			ISmachableAction action = (ISmachableAction) o;
			return actuatorName.equals(action.getActuatorName()) && (value==action.getValue());
		} else {
			return false;
		}
	}

	@Override
	public String toString() {
		return actuatorName + " " + value;
	}
}