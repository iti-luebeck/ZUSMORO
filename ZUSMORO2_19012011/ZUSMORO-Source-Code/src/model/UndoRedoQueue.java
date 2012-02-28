package model;

import java.util.LinkedList;

public class UndoRedoQueue {
	private static final long serialVersionUID = 0;
	private final LinkedList<ChangeEvent> undoList;
	private final LinkedList<ChangeEvent> redoList;
	private boolean locked;

	public UndoRedoQueue() {
		undoList = new LinkedList<ChangeEvent>();
		redoList = new LinkedList<ChangeEvent>();
		locked = false;
	}

	public void clear() {
		undoList.clear();
		redoList.clear();
	}

	public void enque(ChangeEvent e) {
		if (!locked) {
			undoList.addLast(e);
			redoList.clear();
		}
	}

	public ChangeEvent dequeUndoEvent() {
		locked = true;
		ChangeEvent event = undoList.pollLast();
		redoList.addFirst(event);
		return event;
	}

	public ChangeEvent dequeRedoEvent() {
		locked = true;
		ChangeEvent event = redoList.pollFirst();
		undoList.addLast(event);
		return event;
	}

	public void unlock() {
		locked = false;
	}

	public boolean hasUndoEvents() {
		return !undoList.isEmpty();
	}

	public boolean hasRedoEvents() {
		return !redoList.isEmpty();
	}

	public boolean nextUndoIsFollowEvent() {
		return undoList.getLast().followEvent;
	}
}
