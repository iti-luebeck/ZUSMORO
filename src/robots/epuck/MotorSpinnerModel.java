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
package robots.epuck;

import java.util.ArrayList;

import javax.swing.SpinnerModel;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

public class MotorSpinnerModel implements SpinnerModel, ChangeListener {

	private ArrayList<ChangeListener> listeners;
	private MotorModel model;
	
	public MotorSpinnerModel(MotorModel model) {
		this.listeners = new ArrayList<ChangeListener>(1);
		this.model = model;
		this.model.addChangeListener(this);
	}

	public void addChangeListener(ChangeListener listener) {
		listeners.add(listener);
	}

	public Object getNextValue() {
		return new Integer(Math.min(model.getMaximum(), model.getValue()+1));
	}

	public Object getPreviousValue() {
		return new Integer(Math.max(model.getMinimum(), model.getValue()-1));
	}


	public void removeChangeListener(ChangeListener listener) {
		listeners.remove(listener);
	}

	public void setValue(Object value) {
		model.setValue(((Integer)value).intValue());
	}

	public Object getValue() {
		return new Integer(model.getValue());
	}

	public void stateChanged(ChangeEvent e) {
		for (ChangeListener listener : this.listeners) {
			listener.stateChanged(new ChangeEvent(this));
		}
	}
}