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

import javax.swing.BoundedRangeModel;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

public class MotorModel implements BoundedRangeModel {

	private ArrayList<ChangeListener> listeners;
	private boolean isAdjusting;
	private int max;
	private int min;
	private int value;
	
	public MotorModel(int min, int max, int value) {
		this.listeners = new ArrayList<ChangeListener>(2);
		this.isAdjusting = false;
		this.max = max; 
		this.min = min;
		this.value = value;
	}
	
	public void addChangeListener(ChangeListener listener) {
		listeners.add(listener);
	}

	public int getExtent() {
		//System.out.println("MotorModel.getExtent()");
		return 0;
	}

	public int getMaximum() {
		//System.out.println("MotorModel.getMaximum()");
		return max;
	}

	public int getMinimum() {
		//System.out.println("MotorModel.getMinimum()");
		return min;
	}

	public int getValue() {
		//System.out.println("MotorModel.getValue()");
		return value;
	}

	public boolean getValueIsAdjusting() {
		//System.out.println("MotorModel.getValueisAdjusting()");
		return isAdjusting;
	}

	public void removeChangeListener(ChangeListener listener) {
		listeners.remove(listener);
	}

	public void setExtent(int arg0) {
		//System.out.println("MotorModel.setExtent() "+arg0);
	}

	public void setMaximum(int arg0) {
		//System.out.println("MotorModel.setMaximum() "+arg0);
		this.max = arg0;
		this.notifyListeners();
	}

	public void setMinimum(int arg0) {
		//System.out.println("MotorModel.setMinimum() "+arg0);
		this.min = arg0;
		this.notifyListeners();
	}

	public void setRangeProperties(int value, int extend, int min, int max, boolean adjusting) {
		//System.out.println("MotorModel.setRangeProperties()");
		this.value = value;
		this.min = min;
		this.max = max;
		this.isAdjusting = adjusting;
		this.notifyListeners();
	}

	public void setValue(int value) {
		//System.out.println("MotorModel.setValue() "+ value);
		this.value = value;
		this.notifyListeners();
	}

	public void setValueIsAdjusting(boolean adjusting) {
		//System.out.println("MotorModel.setValueIsAdjusting() "+adjusting);
		this.isAdjusting = adjusting;
		this.notifyListeners();
	}
	
	private void notifyListeners() {
		for (ChangeListener listener : this.listeners) {
			listener.stateChanged(new ChangeEvent(this));
		}
	}
}