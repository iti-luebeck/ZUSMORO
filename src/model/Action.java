package model;


public class Action {

	private String key;
	private int value;

	public Action(String key, int value) {
		this.key = key;
		this.value = value;
	}

	public String getKey() {
		return key;
	}

	public void setKey(String key) {
		this.key = key;
	}

	public int getValue() {
		return value;
	}
	public void setValue(int value) {
		this.value = value;
	}

	@Override
	public boolean equals(Object o) {
		if (o instanceof Action) {
			Action action = (Action) o;
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