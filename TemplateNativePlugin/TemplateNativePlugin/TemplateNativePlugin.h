#define DLLExport __declspec(dllexport)

class Test {

public:

	float x = 0;

	void increment() {
		x += 1;
	}

	float get_value() {
		return x;
	}
};


extern "C" {
	DLLExport Test* initialise();
	DLLExport float update(Test*);
}

// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the TEMPLATENATIVEPLUGIN_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// TEMPLATENATIVEPLUGIN_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef TEMPLATENATIVEPLUGIN_EXPORTS
#define TEMPLATENATIVEPLUGIN_API __declspec(dllexport)
#else
#define TEMPLATENATIVEPLUGIN_API __declspec(dllimport)
#endif

// This class is exported from the dll
class TEMPLATENATIVEPLUGIN_API CTemplateNativePlugin {
public:
	CTemplateNativePlugin(void);
	// TODO: add your methods here.
};

extern TEMPLATENATIVEPLUGIN_API int nTemplateNativePlugin;

TEMPLATENATIVEPLUGIN_API int fnTemplateNativePlugin(void);

