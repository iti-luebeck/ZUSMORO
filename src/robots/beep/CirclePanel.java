package robots.beep;

import java.awt.Color;
import java.awt.Graphics;

import javax.swing.JPanel;

public class CirclePanel extends JPanel {

	/**
	 * 
	 */
	private static final long serialVersionUID = -3521303370776086443L;
	private Color color;

	public CirclePanel(Color color) {
		super();
		this.color = color;
		this.setOpaque(false);

	}

	public CirclePanel() {
		this(null);
	}

	@Override
	public void paint(Graphics g) {
		super.paint(g);
		if (color != null) {
			g.setColor(color);
			g.fillArc(0, 0, this.getWidth(), this.getHeight(), 0, 360);
		} else {
			g.drawRect(0, 0, this.getWidth()-1, this.getHeight()-1);
			g.drawLine(0, 0, this.getHeight()-1, this.getWidth()-1);
			g.drawLine(this.getWidth()-1, 0, 0, this.getHeight()-1);	
		}
		this.repaint();
	}

	@Override
	public void setBackground(Color arg0) {
		color = arg0;
		Graphics g = this.getGraphics();
		if (g != null) {
			paint(g);
		}

	}

	public void setIgnore(boolean ignore) {
		Graphics g = this.getGraphics();
		if (g != null) {

		}
	}

	@Override
	public Color getBackground() {
		return color;
	}

}
