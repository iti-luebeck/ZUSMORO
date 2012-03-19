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