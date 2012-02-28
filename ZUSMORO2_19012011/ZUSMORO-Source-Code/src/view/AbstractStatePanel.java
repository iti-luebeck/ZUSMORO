package view;

import java.util.ArrayList;

import javax.swing.JPanel;

import model.Action;

public abstract class AbstractStatePanel extends JPanel {

	private static final long serialVersionUID = 1881770845372862592L;

	public abstract ArrayList<Action> getActions();
}
