/*
 * MousePressCatcher.h
 *
 *  Created on: 25.01.2012
 *      Author: cyborg-x1
 */

#ifndef MOUSEPRESSCATCHER_HPP_
#define MOUSEPRESSCATCHER_HPP_

#include <QObject>
#include <QPoint>
#include <QEvent>
#include <QDebug>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QGraphicsSceneEvent>


class MousePressCatcher: public QObject, public QGraphicsItem
{
	Q_OBJECT
public:
	MousePressCatcher(QObject *parent, QGraphicsScene* scene);
	virtual ~MousePressCatcher();

protected:
	virtual bool sceneEventFilter(QGraphicsItem *watched, QEvent *event);
    virtual void paint(QPainter *,const QStyleOptionGraphicsItem *,QWidget *);
	virtual QRectF boundingRect(void) const;

signals:
	void clickPos(QPointF);
};

#endif /* MOUSEPRESSCATCHER_H_ */
