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
