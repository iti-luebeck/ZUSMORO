/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
