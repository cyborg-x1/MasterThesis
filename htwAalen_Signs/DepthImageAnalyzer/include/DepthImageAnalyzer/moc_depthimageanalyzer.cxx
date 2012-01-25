/****************************************************************************
** Meta object code from reading C++ file 'depthimageanalyzer.hpp'
**
** Created: Wed Jan 25 19:46:49 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "depthimageanalyzer.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'depthimageanalyzer.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_DepthImageAnalyzer__DepthImageAnalyzer[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      44,   40,   39,   39, 0x08,
      63,   39,   39,   39, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_DepthImageAnalyzer__DepthImageAnalyzer[] = {
    "DepthImageAnalyzer::DepthImageAnalyzer\0"
    "\0img\0depthImage(QImage)\0clickPos(QPointF)\0"
};

const QMetaObject DepthImageAnalyzer::DepthImageAnalyzer::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_DepthImageAnalyzer__DepthImageAnalyzer,
      qt_meta_data_DepthImageAnalyzer__DepthImageAnalyzer, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &DepthImageAnalyzer::DepthImageAnalyzer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *DepthImageAnalyzer::DepthImageAnalyzer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *DepthImageAnalyzer::DepthImageAnalyzer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_DepthImageAnalyzer__DepthImageAnalyzer))
        return static_cast<void*>(const_cast< DepthImageAnalyzer*>(this));
    return QWidget::qt_metacast(_clname);
}

int DepthImageAnalyzer::DepthImageAnalyzer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: depthImage((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 1: clickPos((*reinterpret_cast< QPointF(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
