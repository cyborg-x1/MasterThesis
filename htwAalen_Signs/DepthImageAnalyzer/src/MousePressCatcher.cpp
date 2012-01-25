/*
 * MousePressCatcher.cpp
 *
 *  Created on: 25.01.2012
 *      Author: cyborg-x1
 */
#include "DepthImageAnalyzer/MousePressCatcher.hpp"

MousePressCatcher::MousePressCatcher(QObject *parent, QGraphicsScene* scene)
:QObject(parent)
,QGraphicsItem(0,scene)
{
	// TODO Auto-generated constructor stub
}

MousePressCatcher::~MousePressCatcher() {
	// TODO Auto-generated destructor stub
}

bool MousePressCatcher::sceneEventFilter(QGraphicsItem *watched, QEvent *event)
{
	qDebug()<<"Event"<<event->type();
    if (event->type() == QEvent::GraphicsSceneMousePress)
    {
			QGraphicsSceneMouseEvent *buttonEvent = static_cast<QGraphicsSceneMouseEvent *>(event);

			emit clickPos(buttonEvent->pos());
        return true;
    } else {
        // standard event processing
        return QGraphicsItem::sceneEventFilter(watched, event);
    }
}

void MousePressCatcher::paint(QPainter *,const QStyleOptionGraphicsItem *,QWidget *)
{

}

QRectF MousePressCatcher::boundingRect(void) const
{
	        return QRectF(0,0,-1,-1);
}
