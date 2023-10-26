/****************************************************************************
** Meta object code from reading C++ file 'SensorManager.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/
#include <Quarter/SensorManager.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SensorManager.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SIM__Coin3D__Quarter__SensorManager_t {
    QByteArrayData data[8];
    char stringdata0[119];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SIM__Coin3D__Quarter__SensorManager_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SIM__Coin3D__Quarter__SensorManager_t qt_meta_stringdata_SIM__Coin3D__Quarter__SensorManager = {
    {
QT_MOC_LITERAL(0, 0, 35), // "SIM::Coin3D::Quarter::SensorM..."
QT_MOC_LITERAL(1, 36, 11), // "idleTimeout"
QT_MOC_LITERAL(2, 48, 0), // ""
QT_MOC_LITERAL(3, 49, 12), // "delayTimeout"
QT_MOC_LITERAL(4, 62, 17), // "timerQueueTimeout"
QT_MOC_LITERAL(5, 80, 18), // "sensorQueueChanged"
QT_MOC_LITERAL(6, 99, 15), // "setTimerEpsilon"
QT_MOC_LITERAL(7, 115, 3) // "sec"

    },
    "SIM::Coin3D::Quarter::SensorManager\0"
    "idleTimeout\0\0delayTimeout\0timerQueueTimeout\0"
    "sensorQueueChanged\0setTimerEpsilon\0"
    "sec"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SIM__Coin3D__Quarter__SensorManager[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x0a /* Public */,
       3,    0,   40,    2, 0x0a /* Public */,
       4,    0,   41,    2, 0x0a /* Public */,
       5,    0,   42,    2, 0x0a /* Public */,
       6,    1,   43,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double,    7,

       0        // eod
};

void SIM::Coin3D::Quarter::SensorManager::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SensorManager *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->idleTimeout(); break;
        case 1: _t->delayTimeout(); break;
        case 2: _t->timerQueueTimeout(); break;
        case 3: _t->sensorQueueChanged(); break;
        case 4: _t->setTimerEpsilon((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject SIM::Coin3D::Quarter::SensorManager::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_SIM__Coin3D__Quarter__SensorManager.data,
    qt_meta_data_SIM__Coin3D__Quarter__SensorManager,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SIM::Coin3D::Quarter::SensorManager::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SIM::Coin3D::Quarter::SensorManager::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SIM__Coin3D__Quarter__SensorManager.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int SIM::Coin3D::Quarter::SensorManager::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
