package model.onboard;

import java.awt.BorderLayout;
import java.awt.Container;

import robots.epuck.BackgroundPanel;
import robots.epuck.DebugView;

public class OnBoardEPuckDebugView extends DebugView {

	public OnBoardEPuckDebugView(OnBoardEpuck robot) {
		super(robot);
		panel = new OnBoardEPuckBackgroundPanel(BackgroundPanel.debug_background);
		add(panel);
	}

}
