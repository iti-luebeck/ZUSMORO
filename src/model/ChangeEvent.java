package model;

public class ChangeEvent {

	public enum ChangeEventType {
		STATE_CREATE,
		STATE_DELETE,
		STATE_MOVE,
		TRANSITION_CREATE,
		TRANSITION_DELETE,
		TRANSITION_MOVE,
		ACTIVE_STATE_CHANGE;
	}

	public final ChangeEventType type;
	public final Object object;
	public final boolean followEvent;

	public ChangeEvent(ChangeEventType type, Object o, boolean follow) {
		this.type = type;
		this.object = o;
		this.followEvent = follow;
	}

	@Override
	public String toString() {
		return "ChangeEvent: " + type + " " + object;
	}
}