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