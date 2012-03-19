package robots.epuck;

import java.awt.Component;
import java.awt.event.MouseEvent;

import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JSpinner;

import model.ActionController;

public class ControllerPanel {

	private JSpinner offsetSpinner;
	private JSpinner minSpinner;
	private JSpinner maxSpinner;
	private JSpinner kprSpinner;
	private JLabel offset;
	private JLabel min;
	private JLabel max;
	private JLabel kpr;
	private String[] sensors = { "IR0", "IR1", "IR2", "IR3", "IR4", "IR5", "IR6", "IR7", "UIR0", "UIR1", "UIR2" };
	private JLabel sensor1;
	private JComboBox sensor1Box;
	private JCheckBox enableSensor2;
	private JLabel sensor2;
	private JComboBox sensor2Box;
	private JLabel cV;
	private JSpinner cVSpinner;
	private JLabel formel1;
	private JLabel formel2;
	private JLabel formel3;
	private JCheckBox enabled;
	private ActionController ac;

	public ControllerPanel() {
		offset = new JLabel("Offset");
		offsetSpinner = new JSpinner();

		min = new JLabel("Minimum");
		minSpinner = new JSpinner();

		max = new JLabel("Maxmimum");
		maxSpinner = new JSpinner();

		kpr = new JLabel("Kpr");
		kprSpinner = new JSpinner();

		sensor1 = new JLabel("Sensor1");
		sensor1Box = new JComboBox(sensors);

		enableSensor2 = new JCheckBox("Sensor2 verwenden");

		sensor2 = new JLabel("Sensor2");
		sensor2Box = new JComboBox(sensors);

		cV = new JLabel("Sollwert");
		cVSpinner = new JSpinner();

		formel1 = new JLabel("X = (Kpr/1000) * ( Sollwert - (Sensor1() - Sensor2())");
		formel2 = new JLabel("Minimum <= X <= Maximum");
		formel3 = new JLabel("Stellgröße = Offset + X");

		enabled = new JCheckBox("Regler aktivieren");
		ac = null;
	}

	public ActionController showPanel(MouseEvent e,ActionController ac2) {
		setValues(ac2);
		Object[] params = { offset, offsetSpinner, min, minSpinner, max, maxSpinner, kpr, kprSpinner, sensor1,
				sensor1Box, enableSensor2, sensor2, sensor2Box, cV, cVSpinner, formel1, formel2, formel3, enabled };
		int n = JOptionPane.showConfirmDialog((Component) e.getSource(), params, "P-Regler",
				JOptionPane.OK_CANCEL_OPTION);

		if (!enabled.isSelected()) {
			return null;
		} else if (n == 0) {
			ac.setOffset((Integer) offsetSpinner.getValue());
			ac.setMin((Integer) minSpinner.getValue());
			ac.setMax((Integer) maxSpinner.getValue());
			ac.setKpr((Integer) kprSpinner.getValue());
			ac.setVar(enableSensor2.isSelected() ? "DIFFERRENCE_" + (String) sensor1Box.getSelectedItem() + "_"
					+ (String) sensor2Box.getSelectedItem() : (String) sensor1Box.getSelectedItem());
			ac.setCompvalue((Integer) cVSpinner.getValue());
		}
		return ac;

	}

	private void setValues(ActionController ac) {
		this.ac = ac;
		offsetSpinner.setValue(ac.getOffset());
		maxSpinner.setValue(ac.getMax());
		minSpinner.setValue(ac.getMin());
		kprSpinner.setValue(ac.getKpr());
		cVSpinner.setValue(ac.getCompvalue());
		enabled.setSelected(true);

		if (ac.getVar() != null) {
			if (ac.getVar().startsWith("DIFFERENCE")) {
				enableSensor2.setSelected(true);
				String[] split = ac.getVar().split("_");
				sensor1Box.setSelectedItem(split[1]);
				sensor1Box.setSelectedItem(split[2]);
			} else {
				sensor1Box.setSelectedItem(ac.getVar());
			}
		}

	}

}
