package model;

import smachGenerator.ISmachableAction;


public class Action implements ISmachableAction {

	private String key;
	private int value;

	public Action(String key, int value) {
		this.key = key;
		this.value = value;
	}

	/* (non-Javadoc)
	 * @see model.ISmachableAction#getKey()
	 */
	@Override
	public String getKey() {
		return key;
	}

	public void setKey(String key) {
		this.key = key;
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
			return key.equals(action.getKey()) && (value==action.getValue());
		} else {
			return false;
		}
	}

	@Override
	public String toString() {
		return key + " " + value;
	}
}