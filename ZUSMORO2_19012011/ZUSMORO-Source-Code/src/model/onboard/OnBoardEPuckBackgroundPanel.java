package model.onboard;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;

import robots.epuck.BackgroundPanel;

public class OnBoardEPuckBackgroundPanel extends BackgroundPanel {

	public OnBoardEPuckBackgroundPanel(Image i) {
		super(i);
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);

		Graphics2D g2d = (Graphics2D) g;
		if(robot!=null && robot instanceof OnBoardEpuck){
			g2d.setFont(g2d.getFont().deriveFont(Font.BOLD, 14.0f));
			int timer = ((OnBoardEpuck)robot).getTimerState();
			g2d.setColor(Color.BLACK);
			g2d.drawString(timer+"", 170, 262);
		}
	}
}
