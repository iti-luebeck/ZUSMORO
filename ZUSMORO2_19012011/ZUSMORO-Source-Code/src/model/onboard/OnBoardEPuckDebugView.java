package model.onboard;

import robots.epuck.BackgroundPanel;
import robots.epuck.DebugView;

public class OnBoardEPuckDebugView extends DebugView {

	public OnBoardEPuckDebugView(OnBoardEpuck robot) {
		super(robot);
		remove(panel);
		panel = new OnBoardEPuckBackgroundPanel(BackgroundPanel.debug_background);
		panel.setLayout(null);
		add(panel);
	}

}
