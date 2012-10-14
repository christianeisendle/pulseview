/*
 * This file is part of the PulseView project.
 *
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <extdef.h>

#include "signal.h"

namespace pv {

const QSizeF Signal::LabelPadding(4, 0);

Signal::Signal(QString name) :
	_name(name)
{
}

QString Signal::get_name() const
{
	return _name;
}

QRectF Signal::get_label_rect(QPainter &p, const QRect &rect)
{
	const QSizeF text_size = p.boundingRect(
		QRectF(0, 0, rect.width(), 0), 0, _name).size();

	const float nominal_offset = get_nominal_offset(rect);
	const QSizeF label_size(
		text_size.width() + LabelPadding.width() * 2,
		text_size.height() + LabelPadding.height() * 2);
	const float label_arrow_length = label_size.height() / 2;
	return QRectF(
		rect.right() - label_arrow_length - label_size.width(),
		nominal_offset - label_size.height() / 2,
		label_size.width(), label_size.height());
}

void Signal::paint_label(QPainter &p, const QRect &rect)
{
	p.setBrush(get_colour());

	const QColor colour = get_colour();
	const float nominal_offset = get_nominal_offset(rect);
	const QRectF label_rect = get_label_rect(p, rect);

	// Paint the label
	const QPointF points[] = {
		label_rect.topLeft(),
		label_rect.topRight(),
		QPointF(rect.right(), nominal_offset),
		label_rect.bottomRight(),
		label_rect.bottomLeft()
	};

	const QPointF highlight_points[] = {
		QPointF(label_rect.left() + 1, label_rect.top() + 1),
		QPointF(label_rect.right(), label_rect.top() + 1),
		QPointF(rect.right() - 1, nominal_offset),
		QPointF(label_rect.right(), label_rect.bottom() - 1),
		QPointF(label_rect.left() + 1, label_rect.bottom() - 1)
	};

	p.setPen(Qt::transparent);
	p.setBrush(colour);
	p.drawPolygon(points, countof(points));

	p.setPen(colour.lighter());
	p.setBrush(Qt::transparent);
	p.drawPolygon(highlight_points, countof(highlight_points));

	p.setPen(colour.darker());
	p.setBrush(Qt::transparent);
	p.drawPolygon(points, countof(points));

	// Paint the text
	p.setPen((colour.lightness() > 64) ? Qt::black : Qt::white);
	p.drawText(label_rect, Qt::AlignCenter | Qt::AlignVCenter, _name);
}

} // namespace pv
