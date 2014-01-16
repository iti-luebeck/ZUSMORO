package robots.beep;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Point;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;

import javax.swing.ImageIcon;
import javax.swing.JPanel;

import robots.beep.BeepRobot;
import robots.epuck.SensorPanel;
import robots.epuck.ValuePosition;
import model.bool.Variable.Operator;

/**
 * @author ida
 * 
 *         Class setting the epuck image into the background. Different images
 *         for different modes.
 */
public class BeepDebugBackgroundPanel extends JPanel  {

	private static final long serialVersionUID = -278302991810159232L;
	/**
	 * The image of the beep for programming the states
	 */
	public static final Image state_background = new ImageIcon(
			BeepDebugBackgroundPanel.class.getResource("beep-background.png"))
			.getImage();
	/**
	 * The image of the beep for transitions
	 */
	public static final Image trans_background = new ImageIcon(
			BeepDebugBackgroundPanel.class.getResource("beep_trans_background.png"))
			.getImage();
	/**
	 * The image of the beep for debug-mode
	 */
	public static final Image debug_background = new ImageIcon(
			BeepDebugBackgroundPanel.class.getResource("beep_debug_background.png"))
			.getImage();

	private Image background;

	private final int ROBOT_RADIUS = 150;
	private final Point ROBOT_CENTER = new Point(188, 210);

	// Positionen für die Anzeige der Werte (x,y,rotation)
	private ValuePosition[] labelPos = new ValuePosition[11];

	private SensorPanel[] sensorPanels;
	protected BeepRobot robot;

	/**
	 * @param background
	 *            image of epuck in png format
	 * 
	 *            Sets the image of the epuck. Different images for different
	 *            modes (debug, program a state, transition)
	 */
	public BeepDebugBackgroundPanel(Image background) {
		super();
		setPreferredSize(new Dimension(400, 400));
		this.background = background;
		//Sensor Positions

		labelPos[0] = new ValuePosition(358, 276, 0);
		labelPos[1] = new ValuePosition(238, 383, 0);
		labelPos[2] = new ValuePosition(112, 379, 0);
		labelPos[3] = new ValuePosition(6, 277, 0);
		labelPos[4] = new ValuePosition(12, 126, 0);
		labelPos[5] = new ValuePosition(133, 28, 0);
		labelPos[6] = new ValuePosition(249, 33, 0);
		labelPos[7] = new ValuePosition(358, 130, 0);
		
		labelPos[8] = new ValuePosition(141, 135, 0);
		labelPos[9] = new ValuePosition(192, 135, 0);
		labelPos[10] = new ValuePosition(243, 135, 0);
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		if (background == state_background) {
			g.drawImage(BeepDebugBackgroundPanel.state_background, 0, 0, null);
		} else if (background == trans_background) {
			g.drawImage(BeepDebugBackgroundPanel.trans_background, 0, 0, null);
			Graphics2D g2d = (Graphics2D) g;
			// for (int i = 0; i < sensorShapes.length; i++) {
			// g2d.fill(sensorShapes[i]);
			// }
			if (sensorPanels != null) {
				AffineTransform Tx = g2d.getTransform();
				g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
						RenderingHints.VALUE_ANTIALIAS_ON);
				g2d.setColor(Color.WHITE);
				g2d.setFont(g2d.getFont().deriveFont(Font.BOLD, 14.0f));
				Operator op;
				int compValue;
				ValuePosition pos;
				for (int i = 0; i < sensorPanels.length; i++) {
					pos = labelPos[i];
					try {
						op = sensorPanels[i].getOperator();
						compValue = sensorPanels[i].getCompValue();
						if (i == 11) {
							g2d.setColor(Color.BLACK);
							g2d.setFont(g2d.getFont().deriveFont(18.0f));
						}
					} catch (NumberFormatException e) {
						continue;
					}
					Tx.rotate(pos.angle, pos.x, pos.y);
					g2d.setTransform(Tx);
					g2d.drawString(op.toString() + compValue, pos.x, pos.y);
					Tx.rotate(-1 * pos.angle, pos.x, pos.y);
					g2d.setTransform(Tx);
				}
			}
		} else if (background == debug_background) {
			g.drawImage(BeepDebugBackgroundPanel.debug_background, 0, 0, null);
			Graphics2D g2d = (Graphics2D) g;
			if (robot != null) {
				AffineTransform Tx = g2d.getTransform();
				g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
						RenderingHints.VALUE_ANTIALIAS_ON);
				g2d.setColor(Color.WHITE);
				g2d.setFont(g2d.getFont().deriveFont(Font.BOLD, 14.0f));
				// Operator op;
				int value = 0;
				ValuePosition pos;
				for (int i = 0; i < labelPos.length - 1; i++) {
					pos = labelPos[i];
					if (i <= 7) {
						// op = sensorPanels[i].getOperator();
						value = robot.getVariableValue("IR"+i);
					} else if (i <= 10) {
						value = robot.getVariableValue("UIR"+(i - 8));
					}
					// if (i == 11) {
					// g2d.setColor(Color.BLACK);
					// g2d.setFont(g2d.getFont().deriveFont(18.0f));
					// }
					Tx.rotate(pos.angle, pos.x, pos.y);
					g2d.setTransform(Tx);
					g2d.drawString(" " + value, pos.x, pos.y);
					Tx.rotate(-1 * pos.angle, pos.x, pos.y);
					g2d.setTransform(Tx);
				}
			}
		}
	}

	public void update(Object update) {
		if (update instanceof SensorPanel[]) {
			this.sensorPanels = (SensorPanel[]) update;
		} else if (update instanceof BeepRobot) {
			this.robot = (BeepRobot) update;
		}
	}

}