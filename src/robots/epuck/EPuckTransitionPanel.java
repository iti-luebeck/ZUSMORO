/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package robots.epuck;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

import javax.swing.ImageIcon;
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
public class EPuckTransitionPanel extends AbstractTransitionPanel implements
		MouseListener {
	private static final long serialVersionUID = 4115771690354407351L;
	private final Image trans_background = new ImageIcon(
			EPuckTransitionPanel.class.getResource("trans_background.png"))
			.getImage();

	private final int IR0 = 0;
	// private final int IR1 = 1;
	// private final int IR2 = 2;
	// private final int IR3 = 3;
	// private final int IR4 = 4;
	// private final int IR5 = 5;
	// private final int IR6 = 6;
	private final int IR7 = 7;
	private final int UIR0 = 8;
	// private final int UIR1 = 9;
	private final int UIR2 = 10;
	private final int TIMER = 11;

	private SensorPanel[] sensorPanels;
	private final Shape[] sensorShapes = {
			//IR sensors
			new Rectangle2D.Double(229, 33, 31, 38),
			new Rectangle2D.Double(292, 81, 44, 28),
			new Rectangle2D.Double(336, 186, 35, 30),
			new Rectangle2D.Double(262, 320, 33, 30),
			new Rectangle2D.Double(108, 316, 30, 38),
			new Rectangle2D.Double(29, 184, 33, 31),
			new Rectangle2D.Double(60, 83, 38, 28),
			new Rectangle2D.Double(137, 41, 35, 27),
			//Ground Sensors
			new Rectangle2D.Double(134, 102, 32, 40),
			new Rectangle2D.Double(185, 102, 32, 40),
			new Rectangle2D.Double(234, 102, 32, 40),
			//Timer
			new Rectangle2D.Double(160, 238, 80, 32) };

	private ValuePosition[] labelPos = {
			//IR sensors
			new ValuePosition(220, 61, Math.PI / 9.5),
			new ValuePosition(290, 93, Math.PI / 3.5),
			new ValuePosition(339, 180, Math.PI / 2),
			new ValuePosition(257, 340, Math.PI / -5.5),
			new ValuePosition(110, 320, Math.PI / 6),
			new ValuePosition(60, 226, Math.PI / -2),
			new ValuePosition(83, 126, Math.PI / -3.5),
			new ValuePosition(136, 75, Math.PI / -10),
			//Ground sensors
			new ValuePosition(126, 143, 0), 
			new ValuePosition(179, 143, 0),
			new ValuePosition(232, 143, 0),
			//Timer
			new ValuePosition(165, 260, 0) };

	private Transition transition;
	private DifferencePanel differencePanel;

	/**
	 * @param trans
	 *            Name of the transition
	 * 
	 *            Constructor for the TransitionPanel. Adds Listeners, sets
	 *            initial values.
	 */
	public EPuckTransitionPanel(Transition trans) {
		this.transition = trans;
		addMouseListener(this);
		setLayout(null);
		initComponents();
		setValues();
		setPreferredSize(new Dimension(400, 600));
	}

	private void initComponents() {
		sensorPanels = new SensorPanel[12];
		for (int i = 0; i < sensorPanels.length; i++) {
			sensorPanels[i] = new SensorPanel();
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
						sensorPanels[index + UIR0].setOperator(variable
								.getOperator());
						sensorPanels[index + UIR0].setCompValue(variable
								.getCompValue());
					} catch (Exception e) {
						System.out.println("HANDLED EXCEPTION:");
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
		for (int i = UIR0; i <= UIR2; i++) {
			sensor = sensorPanels[i];
			try {
				newGuard.addOperand(new Variable("UIR" + (i - UIR0), sensor
						.getOperator(), sensor.getCompValue()));
				toolTip.append("<br>UIR" + (i - UIR0)
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
				addPanel(sensorPanels[i], x, y);
				return;
			}
		}
		removeAll();
		revalidate();
		repaint();

	}

	private void addPanel(JPanel panelToAdd, int x, int y) {
		Dimension prefSize = panelToAdd.getPreferredSize();
		panelToAdd.setBounds(x - 45, y - 19, prefSize.width, prefSize.height);
		removeAll();
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
