TEMPLATE = lib

HEADERS += \
    CIProxie.h \
    ANGConProxie.h \
    AKIProxie.h \
    AAPI.h \
    AAPI_Util.h

SOURCES += \
    AAPI.cxx

win32 {

} macx {
	LIBS += /Applications/"Aimsun Next.app"/Contents/PlugIns/liba2kernel.dylib
	LIBS += /Applications/"Aimsun Next.app"/Contents/PlugIns/libacontrol.dylib
} linux {
    LIBS += -L/opt/Aimsun_8_2_0/ -la2kernel -lacontrol
}
