/****************************************************************************
** Meta object code from reading C++ file 'qnode.hpp'
**
** Created: Wed Jan 25 23:41:18 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "qnode.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qnode.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_DepthImageAnalyzer__QNode[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      27,   26,   26,   26, 0x05,
      44,   26,   26,   26, 0x05,
      58,   26,   26,   26, 0x05,
      74,   72,   26,   26, 0x05,

 // slots: signature, parameters, type, tag, flags
     108,  104,   26,   26, 0x0a,
     126,   26,   26,   26, 0x0a,
     157,  145,   26,   26, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_DepthImageAnalyzer__QNode[] = {
    "DepthImageAnalyzer::QNode\0\0loggingUpdated()\0"
    "rosShutdown()\0image(QImage)\0,\0"
    "currentPosValue(QPoint,float)\0pos\0"
    "clickPos(QPointF)\0highlightDisable()\0"
    "lower,upper\0highlight(float,float)\0"
};

const QMetaObject DepthImageAnalyzer::QNode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_DepthImageAnalyzer__QNode,
      qt_meta_data_DepthImageAnalyzer__QNode, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &DepthImageAnalyzer::QNode::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *DepthImageAnalyzer::QNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *DepthImageAnalyzer::QNode::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_DepthImageAnalyzer__QNode))
        return static_cast<void*>(const_cast< QNode*>(this));
    return QThread::qt_metacast(_clname);
}

int DepthImageAnalyzer::QNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: loggingUpdated(); break;
        case 1: rosShutdown(); break;
        case 2: image((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 3: currentPosValue((*reinterpret_cast< QPoint(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 4: clickPos((*reinterpret_cast< QPointF(*)>(_a[1]))); break;
        case 5: highlightDisable(); break;
        case 6: highlight((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        default: ;
        }
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void DepthImageAnalyzer::QNode::loggingUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void DepthImageAnalyzer::QNode::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void DepthImageAnalyzer::QNode::image(QImage _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void DepthImageAnalyzer::QNode::currentPosValue(QPoint _t1, float _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
