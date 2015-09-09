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
import java.awt.geom.AffineTransform;

import javax.swing.ImageIcon;
import javax.swing.JPanel;

import model.bool.Variable.Operator;

/**
 * @author ida
 * 
 *         Class setting the epuck image into the background. Different images
 *         for different modes.
 */
public class BackgroundPanel extends JPanel {

	private static final long serialVersionUID = -278302991810159232L;
	/**
	 * The image of the epuck for programming the states
	 */
	public static final Image state_background = new ImageIcon(
			BackgroundPanel.class.getResource("e-puck-background.png"))
			.getImage();
	/**
	 * The image of the epuck for transitions
	 */
	public static final Image trans_background = new ImageIcon(
			BackgroundPanel.class.getResource("trans_background.png"))
			.getImage();
	/**
	 * The image of the epuck for debug-mode
	 */
	public static final Image debug_background = new ImageIcon(
			BackgroundPanel.class.getResource("debug_background.png"))
			.getImage();

	private Image background;

	// public Shape[] sensorShapes = { new Rectangle2D.Double(137, 41, 35, 27),
	// new Rectangle2D.Double(60, 83, 38, 28),
	// new Rectangle2D.Double(29, 184, 33, 31), new Rectangle2D.Double(108, 316,
	// 30, 38),
	// new Rectangle2D.Double(262, 320, 33, 30), new Rectangle2D.Double(336,
	// 186, 35, 30),
	// new Rectangle2D.Double(292, 81, 44, 28), new Rectangle2D.Double(229, 33,
	// 31, 38),
	// new Rectangle2D.Double(134, 102, 32, 40), new Rectangle2D.Double(185,
	// 102, 32, 40),
	// new Rectangle2D.Double(234, 102, 32, 40), new Rectangle2D.Double(160,
	// 238, 80, 32) };

	// Positionen für die Anzeige der Werte (x,y,rotation)
	private ValuePosition[] labelPos = {
			new ValuePosition(220, 61, Math.PI / 9.5),
			new ValuePosition(290, 93, Math.PI / 3.5),
			new ValuePosition(339, 180, Math.PI / 2),
			new ValuePosition(257, 340, Math.PI / -5.5),
			new ValuePosition(110, 320, Math.PI / 6),
			new ValuePosition(60, 226, Math.PI / -2),
			new ValuePosition(83, 126, Math.PI / -3.5),
			new ValuePosition(136, 75, Math.PI / -10),
			new ValuePosition(126, 143, 0), new ValuePosition(179, 143, 0),
			new ValuePosition(232, 143, 0), new ValuePosition(165, 260, 0) };

	private SensorPanel[] sensorPanels;
	protected EPuckSensorI robot;

	/**
	 * @param background
	 *            image of epuck in png format
	 * 
	 *            Sets the image of the epuck. Different images for different
	 *            modes (debug, program a state, transition)
	 */
	public BackgroundPanel(Image background) {
		super();
		setPreferredSize(new Dimension(400, 400));
		this.background = background;
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		if (background == state_background) {
			g.drawImage(BackgroundPanel.state_background, 0, 0, null);
		} else if (background == trans_background) {
			g.drawImage(BackgroundPanel.trans_background, 0, 0, null);
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
			g.drawImage(BackgroundPanel.debug_background, 0, 0, null);
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
						value = robot.getIrDistances()[i];
					} else if (i <= 10) {
						value = robot.getFloorIr()[i - 8];
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
		} else if (update instanceof EPuckSensorI) {
			this.robot = (EPuckSensorI) update;
		}
	}
}