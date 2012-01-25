/****************************************************************************
** Meta object code from reading C++ file 'depthimageanalyzer.hpp'
**
** Created: Wed Jan 25 23:41:19 2012
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
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      40,   39,   39,   39, 0x05,
      71,   59,   39,   39, 0x05,

 // slots: signature, parameters, type, tag, flags
      94,   39,   39,   39, 0x08,
     132,   39,   39,   39, 0x08,
     176,  172,   39,   39, 0x08,
     202,  195,   39,   39, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_DepthImageAnalyzer__DepthImageAnalyzer[] = {
    "DepthImageAnalyzer::DepthImageAnalyzer\0"
    "\0highlightDisable()\0lower,upper\0"
    "highlight(float,float)\0"
    "on_pushButton_Highlight_Set_clicked()\0"
    "on_pushButton_Highlight_Reset_clicked()\0"
    "img\0depthImage(QImage)\0,value\0"
    "clickPos(QPoint,float)\0"
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
        case 0: highlightDisable(); break;
        case 1: highlight((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 2: on_pushButton_Highlight_Set_clicked(); break;
        case 3: on_pushButton_Highlight_Reset_clicked(); break;
        case 4: depthImage((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 5: clickPos((*reinterpret_cast< QPoint(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        default: ;
        }
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void DepthImageAnalyzer::DepthImageAnalyzer::highlightDisable()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void DepthImageAnalyzer::DepthImageAnalyzer::highlight(float _t1, float _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
