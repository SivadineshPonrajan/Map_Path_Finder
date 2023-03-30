/****************************************************************************
** Meta object code from reading C++ file 'view.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../chip/view.h"
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'view.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
namespace {
struct qt_meta_stringdata_GraphicsView_t {
    uint offsetsAndSizes[2];
    char stringdata0[13];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_GraphicsView_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_GraphicsView_t qt_meta_stringdata_GraphicsView = {
    {
        QT_MOC_LITERAL(0, 12)   // "GraphicsView"
    },
    "GraphicsView"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_GraphicsView[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

Q_CONSTINIT const QMetaObject GraphicsView::staticMetaObject = { {
    QMetaObject::SuperData::link<QGraphicsView::staticMetaObject>(),
    qt_meta_stringdata_GraphicsView.offsetsAndSizes,
    qt_meta_data_GraphicsView,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_GraphicsView_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<GraphicsView, std::true_type>
    >,
    nullptr
} };

void GraphicsView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    (void)_o;
    (void)_id;
    (void)_c;
    (void)_a;
}

const QMetaObject *GraphicsView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GraphicsView::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_GraphicsView.stringdata0))
        return static_cast<void*>(this);
    return QGraphicsView::qt_metacast(_clname);
}

int GraphicsView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGraphicsView::qt_metacall(_c, _id, _a);
    return _id;
}
namespace {
struct qt_meta_stringdata_View_t {
    uint offsetsAndSizes[38];
    char stringdata0[5];
    char stringdata1[7];
    char stringdata2[1];
    char stringdata3[8];
    char stringdata4[9];
    char stringdata5[6];
    char stringdata6[10];
    char stringdata7[10];
    char stringdata8[22];
    char stringdata9[12];
    char stringdata10[18];
    char stringdata11[19];
    char stringdata12[6];
    char stringdata13[11];
    char stringdata14[12];
    char stringdata15[12];
    char stringdata16[12];
    char stringdata17[19];
    char stringdata18[6];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_View_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_View_t qt_meta_stringdata_View = {
    {
        QT_MOC_LITERAL(0, 4),  // "View"
        QT_MOC_LITERAL(5, 6),  // "zoomIn"
        QT_MOC_LITERAL(12, 0),  // ""
        QT_MOC_LITERAL(13, 7),  // "zoomOut"
        QT_MOC_LITERAL(21, 8),  // "zoomInBy"
        QT_MOC_LITERAL(30, 5),  // "level"
        QT_MOC_LITERAL(36, 9),  // "zoomOutBy"
        QT_MOC_LITERAL(46, 9),  // "resetView"
        QT_MOC_LITERAL(56, 21),  // "setResetButtonEnabled"
        QT_MOC_LITERAL(78, 11),  // "setupMatrix"
        QT_MOC_LITERAL(90, 17),  // "togglePointerMode"
        QT_MOC_LITERAL(108, 18),  // "toggleAntialiasing"
        QT_MOC_LITERAL(127, 5),  // "print"
        QT_MOC_LITERAL(133, 10),  // "rotateLeft"
        QT_MOC_LITERAL(144, 11),  // "rotateRight"
        QT_MOC_LITERAL(156, 11),  // "mapSelected"
        QT_MOC_LITERAL(168, 11),  // "mapResetted"
        QT_MOC_LITERAL(180, 18),  // "onComboBoxSelected"
        QT_MOC_LITERAL(199, 5)   // "index"
    },
    "View",
    "zoomIn",
    "",
    "zoomOut",
    "zoomInBy",
    "level",
    "zoomOutBy",
    "resetView",
    "setResetButtonEnabled",
    "setupMatrix",
    "togglePointerMode",
    "toggleAntialiasing",
    "print",
    "rotateLeft",
    "rotateRight",
    "mapSelected",
    "mapResetted",
    "onComboBoxSelected",
    "index"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_View[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,  104,    2, 0x0a,    1 /* Public */,
       3,    0,  105,    2, 0x0a,    2 /* Public */,
       4,    1,  106,    2, 0x0a,    3 /* Public */,
       6,    1,  109,    2, 0x0a,    5 /* Public */,
       7,    0,  112,    2, 0x08,    7 /* Private */,
       8,    0,  113,    2, 0x08,    8 /* Private */,
       9,    0,  114,    2, 0x08,    9 /* Private */,
      10,    0,  115,    2, 0x08,   10 /* Private */,
      11,    0,  116,    2, 0x08,   11 /* Private */,
      12,    0,  117,    2, 0x08,   12 /* Private */,
      13,    0,  118,    2, 0x08,   13 /* Private */,
      14,    0,  119,    2, 0x08,   14 /* Private */,
      15,    0,  120,    2, 0x08,   15 /* Private */,
      16,    0,  121,    2, 0x08,   16 /* Private */,
      17,    1,  122,    2, 0x08,   17 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   18,

       0        // eod
};

Q_CONSTINIT const QMetaObject View::staticMetaObject = { {
    QMetaObject::SuperData::link<QFrame::staticMetaObject>(),
    qt_meta_stringdata_View.offsetsAndSizes,
    qt_meta_data_View,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_View_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<View, std::true_type>,
        // method 'zoomIn'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'zoomOut'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'zoomInBy'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'zoomOutBy'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'resetView'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'setResetButtonEnabled'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'setupMatrix'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'togglePointerMode'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'toggleAntialiasing'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'print'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'rotateLeft'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'rotateRight'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'mapSelected'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'mapResetted'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onComboBoxSelected'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>
    >,
    nullptr
} };

void View::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<View *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->zoomIn(); break;
        case 1: _t->zoomOut(); break;
        case 2: _t->zoomInBy((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 3: _t->zoomOutBy((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 4: _t->resetView(); break;
        case 5: _t->setResetButtonEnabled(); break;
        case 6: _t->setupMatrix(); break;
        case 7: _t->togglePointerMode(); break;
        case 8: _t->toggleAntialiasing(); break;
        case 9: _t->print(); break;
        case 10: _t->rotateLeft(); break;
        case 11: _t->rotateRight(); break;
        case 12: _t->mapSelected(); break;
        case 13: _t->mapResetted(); break;
        case 14: _t->onComboBoxSelected((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject *View::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *View::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_View.stringdata0))
        return static_cast<void*>(this);
    return QFrame::qt_metacast(_clname);
}

int View::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 15;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
