package robots.beep;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.util.Iterator;

import javax.swing.ImageIcon;
import javax.swing.JColorChooser;
import javax.swing.JLabel;
import javax.swing.JPanel;

import model.Transition;
import model.bool.BooleanExpression;
import model.bool.HugeAnd;
import model.bool.Variable;
import model.bool.Variable.Operator;
import view.AbstractTransitionPanel;

/**
 * @author ida
 * 
 *         Panel to describe the condition for the transition
 */
public class BeepTransitionPanel extends AbstractTransitionPanel implements
		MouseListener {
	private static final long serialVersionUID = 4115771690354407351L;
	private final Image trans_background = new ImageIcon(
			BeepTransitionPanel.class.getResource("beep_trans_background.png"))
			.getImage();

	private final int IR0 = 0;
	private final int IR7 = 7;
	private final int TIMER = 8;

	private SensorPanel[] sensorPanels;
	private CirclePanel[] groundSensors = new CirclePanel[3];
	private Shape[] sensorShapes = new Shape[9];
	private ValuePosition[] labelPos = new ValuePosition[12];

	private Transition transition;
	private DifferencePanel differencePanel;

	/**
	 * @param trans
	 *            Name of the transition
	 * 
	 *            Constructor for the TransitionPanel. Adds Listeners, sets
	 *            initial values.
	 */
	public BeepTransitionPanel(Transition trans) {

		this.transition = trans;
		addMouseListener(this);
		setLayout(null);
		initComponents();
		setValues();
		setPreferredSize(new Dimension(400, 600));
	}

	private void initComponents() {
		// positions of IR sensors and value-labels

		sensorShapes[0] = new Rectangle(0, 0, 0, 0);

		sensorShapes[0] = new Rectangle(360, 255, 30, 30);
		sensorShapes[1] = new Rectangle(243, 360, 30, 30);
		sensorShapes[2] = new Rectangle(115, 355, 30, 30);
		sensorShapes[3] = new Rectangle(16, 253, 30, 30);
		sensorShapes[4] = new Rectangle(16, 110, 30, 30);
		sensorShapes[5] = new Rectangle(135, 8, 30, 30);
		sensorShapes[6] = new Rectangle(252, 13, 30, 30);
		sensorShapes[7] = new Rectangle(360, 110, 30, 30);

		labelPos[0] = new ValuePosition(358, 276, 0);
		labelPos[1] = new ValuePosition(238, 383, 0);
		labelPos[2] = new ValuePosition(112, 379, 0);
		labelPos[3] = new ValuePosition(6, 277, 0);
		labelPos[4] = new ValuePosition(12, 126, 0);
		labelPos[5] = new ValuePosition(133, 28, 0);
		labelPos[6] = new ValuePosition(249, 33, 0);
		labelPos[7] = new ValuePosition(358, 130, 0);
		
		// TIMER
		sensorShapes[8] = new Rectangle(155, 190, 75, 75);
		labelPos[8] = new ValuePosition(168, 245, 0);
		
		// UIR Sensor Pos
		labelPos[9] = new ValuePosition(141, 135, 0);
		labelPos[10] = new ValuePosition(192, 135, 0);
		labelPos[11] = new ValuePosition(243, 135, 0);

		// IR and Timer
		sensorPanels = new SensorPanel[9];
		for (int i = 0; i < sensorPanels.length; i++) {
			sensorPanels[i] = new SensorPanel();
		}
		// groundSensors
		for (int i = 0; i < 3; i++) {
			CirclePanel c = new CirclePanel();
			ValuePosition pos = labelPos[i + 9];
			c.setBounds(pos.x, pos.y, 15, 15);
			c.addMouseListener(new MouseListener() {

				@Override
				public void mouseReleased(MouseEvent e) {
				}

				@Override
				public void mousePressed(MouseEvent e) {
				}

				@Override
				public void mouseExited(MouseEvent e) {
				}

				@Override
				public void mouseEntered(MouseEvent e) {
				}

				@Override
				public void mouseClicked(MouseEvent e) {
					Color col = JColorChooser.showDialog(
							BeepTransitionPanel.this, "Wähle eine Farbe", e
									.getComponent().getBackground());
					e.getComponent().setBackground(col);
				}
			});
			groundSensors[i] = c;
			add(c);
		}

		differencePanel = new DifferencePanel(transition);
	}

	private void setValues() {
		if (transition.getGuard() instanceof HugeAnd) {
			HugeAnd guard = (HugeAnd) transition.getGuard();
			BooleanExpression[] variables = guard.getOperands();
			Variable variable;
			for (int i = 0; i < variables.length; i++) {
				if (variables[i] instanceof Variable) {
					variable = (Variable) variables[i];
				} else {
					continue;
				}
				String name = variable.getVariableName();
				int index = 0;
				if (name.startsWith("IR")) {
					try {
						index = Character.getNumericValue(name.charAt(2));
						sensorPanels[index].setOperator(variable.getOperator());
						sensorPanels[index].setCompValue(variable
								.getCompValue());
					} catch (Exception e) {
						System.out.println("HANDLED EXCEPTION:");
						e.printStackTrace();
						continue;
					}
				} else if (name.startsWith("UIR")) {
					try {
						index = Character.getNumericValue(name.charAt(3));
						Color col = new Color(variable.getCompValue());
						groundSensors[index].setBackground(col);
					} catch (Exception e) {
						System.err.println("HANDLED EXCEPTION:");
						e.printStackTrace();
						continue;
					}
				} else if (name.equalsIgnoreCase("timer")) {
					try {
						sensorPanels[TIMER].setOperator(variable.getOperator());
						sensorPanels[TIMER].setCompValue(variable
								.getCompValue());
					} catch (Exception e) {
						System.out.println("HANDLED EXCEPTION:");
						e.printStackTrace();
						continue;
					}
				}
			}
		}
	}

	@Override
	public BooleanExpression getGuard() {
		HugeAnd newGuard = new HugeAnd();
		SensorPanel sensor;
		boolean somethingSet = false;
		StringBuilder toolTip = new StringBuilder("<html><b>"
				+ transition.getLabel() + "</b>");
		for (int i = IR0; i <= IR7; i++) {
			sensor = sensorPanels[i];
			try {
				newGuard.addOperand(new Variable("IR" + i,
						sensor.getOperator(), sensor.getCompValue()));
				toolTip.append("<br>IR" + i
						+ sensor.getOperator().toHTMLString()
						+ sensor.getCompValue());
				somethingSet = true;
			} catch (Exception e) {
				// don't care
			}
		}
		sensor = sensorPanels[TIMER];
		try {
			newGuard.addOperand(new Variable("timer", sensor.getOperator(),
					sensor.getCompValue()));
			toolTip.append("<br>timer" + sensor.getOperator().toHTMLString()
					+ sensor.getCompValue());
			somethingSet = true;
		} catch (Exception e) {
			// don't care
		}
		for (int i = 0; i <= 2; i++) {
			try {
				if (groundSensors[i].getBackground() != null) {
					newGuard.addOperand(new Variable("UIR" + (i),
							Operator.EQUAL, groundSensors[i].getBackground()
									.getRGB()));
					toolTip.append("<br>UIR" + (i)
							+ Operator.EQUAL.toHTMLString()
							+ groundSensors[i].getBackground());
					somethingSet = true;
				}
			} catch (Exception e) {
				// don't care
			}
		}

		for (Variable v : differencePanel.getVars()) {
			newGuard.addOperand(v);
			String[] split = v.getVariableName().split("_");
			toolTip.append("<br>" + split[1] + "-" + split[2]
					+ v.getOperator().toHTMLString() + v.getCompValue());
			somethingSet = true;
		}

		if (!somethingSet) {
			toolTip.append("<br>true");
		}
		toolTip.append("</html>");
		transition.setToolTipText(toolTip.toString());
		return newGuard;
	}

	public void mouseClicked(MouseEvent e) {

		int x = e.getX();
		int y = e.getY();
		Point2D.Double point = new Point2D.Double(x, y);
		for (int i = 0; i < sensorShapes.length; i++) {
			if (sensorShapes[i].contains(point)) {
				addPanel(sensorPanels[i], Math.min(Math.max(x,50),350) , Math.min(Math.max(y,27),373));
				return;
			}
		}
		removeAll();
		revalidate();
		repaint();

		for (CirclePanel c : groundSensors) {
			if (c != null)
				add(c);
		}

	}

	private void addPanel(JPanel panelToAdd, int x, int y) {
		Dimension prefSize = panelToAdd.getPreferredSize();
		panelToAdd.setBounds(x - 45, y - 19, prefSize.width, prefSize.height);
		removeAll();
		for (CirclePanel c : groundSensors) {
			if (c != null)
				add(c);
		}
		add(panelToAdd);
		revalidate();
		repaint();
	}

	@Override
	protected void paintComponent(Graphics g) {

		differencePanel.setBounds(0, 400, 400, 400);
		add(differencePanel);
		differencePanel.revalidate();
		differencePanel.repaint();
		super.paintComponent(g);
		g.drawImage(trans_background, 0, 0, null);
		Graphics2D g2d = (Graphics2D) g;
		if (sensorPanels != null) {
			AffineTransform Tx = g2d.getTransform();
			g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
					RenderingHints.VALUE_ANTIALIAS_ON);
			g2d.setColor(Color.BLACK);
			g2d.setFont(g2d.getFont().deriveFont(Font.BOLD, 14.0f));
			Operator op;
			int compValue;
			ValuePosition pos;
			for (int i = 0; i < sensorPanels.length; i++) {
				pos = labelPos[i];
				try {
					op = sensorPanels[i].getOperator();
					compValue = sensorPanels[i].getCompValue();
				} catch (NumberFormatException e) {
					continue;
				}
				Tx.rotate(pos.angle, pos.x, pos.y);
				g2d.setTransform(Tx);
				if (i == TIMER) {
					g2d.setColor(Color.BLACK);
					g2d.setFont(g2d.getFont().deriveFont(15.0f).deriveFont(Font.BOLD));
					g2d.drawString(op.toString() + compValue + "ms", pos.x, pos.y);
				}else{
					g2d.drawString(op.toString() + compValue, pos.x, pos.y);
				}
				Tx.rotate(-1 * pos.angle, pos.x, pos.y);
				g2d.setTransform(Tx);
			}
		}

	}

	public void mouseEntered(MouseEvent arg0) {
		// ignore
	}

	public void mouseExited(MouseEvent arg0) {
		// ignore
	}

	public void mousePressed(MouseEvent arg0) {
		// ignore
	}

	public void mouseReleased(MouseEvent arg0) {
		// ignore
	}

}
