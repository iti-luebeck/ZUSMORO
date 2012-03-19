package robots.epuck;

import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.JComboBox;
import javax.swing.JPanel;
import javax.swing.JTextField;

import model.bool.Variable;
import model.bool.Variable.Operator;

public class SensorPanel extends JPanel implements ActionListener {

	private static final long serialVersionUID = 1465749684105013561L;

	private JComboBox opSelect;
	private JTextField compValue;

	public SensorPanel() {
		super();
		setBorder(BorderFactory.createLineBorder(Color.BLACK));
		opSelect = new JComboBox(Variable.Operator.values());
		opSelect.setPreferredSize(new Dimension(50, 27));
		compValue = new JTextField(3);
		compValue.addActionListener(this);
		add(opSelect);
		add(compValue);
	}

	public int getCompValue() {
		return Integer.parseInt(compValue.getText());
	}

	public void setCompValue(int compValue) throws NumberFormatException {
		this.compValue.setText(Integer.toString(compValue));
	}

	public Operator getOperator() {
		return (Operator) opSelect.getSelectedItem();
	}

	public void setOperator(Operator op) {
		opSelect.setSelectedItem(op);
	}

	public String getLabel() {
		return opSelect.getSelectedItem().toString() + " " + compValue.getText();
	}

	public void actionPerformed(ActionEvent e) {
		Container parent = this.getParent();
		parent.remove(this);
		parent.validate();
		parent.repaint();
	}
}
