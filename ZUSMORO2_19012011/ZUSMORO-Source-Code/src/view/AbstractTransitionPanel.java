package view;

import javax.swing.JPanel;

import model.bool.BooleanExpression;

public abstract class AbstractTransitionPanel extends JPanel {

	private static final long serialVersionUID = -4765305635598574384L;

	public abstract BooleanExpression getGuard();
}
