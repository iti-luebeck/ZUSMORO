package robots.beep;

import java.awt.Color;
import java.awt.Graphics;

import javax.swing.JPanel;

public class CirclePanel extends JPanel {

	Color color;
	
	public CirclePanel(Color color){
		super();
		this.color = color;
		this.setOpaque(false);
		
	}
	
	public CirclePanel(){
		this(Color.black);
	}

	@Override
	public void paint(Graphics g){
		super.paint(g);
		g.setColor(color);
		g.fillArc(0, 0, this.getWidth(), this.getHeight(), 0, 360);
	}
	
}