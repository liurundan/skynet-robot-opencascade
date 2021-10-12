QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++20

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    gui_controls.cpp \
    gui_positions.cpp \
    libhal/halio.cpp \
    jog.cpp \
    libkinematic/kinematic.cpp \
    libdxfrw/drw_classes.cpp \
    libdxfrw/drw_entities.cpp \
    libdxfrw/drw_header.cpp \
    libdxfrw/drw_objects.cpp \
    libdxfrw/dx_iface.cpp \
    libdxfrw/intern/drw_dbg.cpp \
    libdxfrw/intern/drw_textcodec.cpp \
    libdxfrw/intern/dwgbuffer.cpp \
    libdxfrw/intern/dwgreader.cpp \
    libdxfrw/intern/dwgreader15.cpp \
    libdxfrw/intern/dwgreader18.cpp \
    libdxfrw/intern/dwgreader21.cpp \
    libdxfrw/intern/dwgreader24.cpp \
    libdxfrw/intern/dwgreader27.cpp \
    libdxfrw/intern/dwgutil.cpp \
    libdxfrw/intern/dxfreader.cpp \
    libdxfrw/intern/dxfwriter.cpp \
    libdxfrw/intern/rscodec.cpp \
    libdxfrw/libdwgr.cpp \
    libdxfrw/libdxfrw.cpp \
    libdxfrw/libdxfrw_functions.cpp \
    libmotion/create_program.cpp \
    libmotion/create_stream.cpp \
    libocct/draw_primitives.cpp \
    libocct/opencascade.cpp \
    libspline/bezier_spline.cpp \
    libspline/cubic_spline.cpp \
    libspline/spline.cpp \
    main.cpp \
    mainwindow.cpp \
    libscurve/scurve.cpp \
    variable.cpp

HEADERS += \
    gui_controls.h \
    gui_positions.h \
    libhal/halio.h \
    jog.h \
    libkinematic/kinematic.h \
    libdxfrw/drw_base.h \
    libdxfrw/drw_classes.h \
    libdxfrw/drw_entities.h \
    libdxfrw/drw_header.h \
    libdxfrw/drw_interface.h \
    libdxfrw/drw_objects.h \
    libdxfrw/dx_data.h \
    libdxfrw/dx_iface.h \
    libdxfrw/intern/drw_cptable932.h \
    libdxfrw/intern/drw_cptable936.h \
    libdxfrw/intern/drw_cptable949.h \
    libdxfrw/intern/drw_cptable950.h \
    libdxfrw/intern/drw_cptables.h \
    libdxfrw/intern/drw_dbg.h \
    libdxfrw/intern/drw_textcodec.h \
    libdxfrw/intern/dwgbuffer.h \
    libdxfrw/intern/dwgreader.h \
    libdxfrw/intern/dwgreader15.h \
    libdxfrw/intern/dwgreader18.h \
    libdxfrw/intern/dwgreader21.h \
    libdxfrw/intern/dwgreader24.h \
    libdxfrw/intern/dwgreader27.h \
    libdxfrw/intern/dwgutil.h \
    libdxfrw/intern/dxfreader.h \
    libdxfrw/intern/dxfwriter.h \
    libdxfrw/intern/rscodec.h \
    libdxfrw/libdwgr.h \
    libdxfrw/libdxfrw.h \
    libdxfrw/libdxfrw_functions.h \
    libdxfrw/main_doc.h \
    libmotion/create_program.h \
    libmotion/create_stream.h \
    libocct/draw_primitives.h \
    libocct/opencascade.h \
    libscurve/scurve.h \
    libspline/bezier_spline.h \
    libspline/cubic_spline.h \
    libspline/spline.h \
    mainwindow.h \
    variable.h

FORMS += \
    gui_controls.ui \
    gui_positions.ui \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH +=  /usr/local/include/opencascade/ \
                /usr/local/include/kdl/ \
                /usr/include/eigen3/ \
                /usr/local/lib/ \
               #../../Downloads/skynet_soft-1.0/opencascade_cad/opencascade-7.4.0/inc/ \
               #../../Downloads/skynet_soft-1.0/kdl_kinematics/orocos_kdl/src/ \

LIBS += -L/usr/local/lib/ \

# project
INCLUDEPATH+=   libspline/ \
                libocct/ \
                libdxfrw/ \
                libscurve/ \
                libhal/ \
                libkinematic/ \
                libmotion/

# lcnc
INCLUDEPATH +=  /usr/lib/ \
                /opt/lib/linuxcnc/modules/ \
                /opt/include/ \
                /usr/include/linuxcnc/

#Hal
INCLUDEPATH+=   /opt/linuxcnc/include/ \
                /opt/linuxcnc/src/hal/ \


LIBS+= -L/opt/linuxcnc/lib/
LIBS+= -llinuxcnchal

#Opencascade
LIBS+= -L/usr/local/lib/ \



LIBS += -lorocos-kdl -llinuxcnchal -Iinclude -Isrc/emc/rs274ngc -Llib -lnml -llinuxcnc -llinuxcnchal -llinuxcncini -lposemath

# Opencascade
LIBS += -lTKPrim
LIBS += -lTKernel -lTKMath -lTKTopAlgo -lTKService
LIBS += -lTKG2d -lTKG3d -lTKV3d -lTKOpenGl
LIBS += -lTKBRep -lTKXSBase -lTKGeomBase
LIBS += -lTKMeshVS -lTKXSDRAW
LIBS += -lTKLCAF -lTKXCAF -lTKCAF
LIBS += -lTKCDF -lTKBin -lTKBinL -lTKBinXCAF -lTKXml -lTKXmlL -lTKXmlXCAF
# -- IGES support
LIBS += -lTKIGES
# -- STEP support
LIBS += -lTKSTEP -lTKXDESTEP -lTKXDEIGES
# -- STL support
LIBS += -lTKSTL
# -- OBJ/glTF support

LIBS += -lTKRWMesh

#src/base/io_occ_base_mesh.cpp \
#src/base/io_occ_gltf.cpp \
#src/base/io_occ_obj.cpp

# -- VRML support
LIBS += -lTKVRML

# this copies the configuration files etc to the build direcory. So user has only to edit the source directory.
copydata.commands = $(COPY_DIR) $$PWD/* $$OUT_PWD
first.depends = $(first) copydata
export(first.depends)
export(copydata.commands)
QMAKE_EXTRA_TARGETS += first copydata

DISTFILES += \
    config/ethercat-conf.xml \
    config/ethercat.hal \
    libdxfrw/main.txt

RESOURCES += \
    config/icons.qrc
